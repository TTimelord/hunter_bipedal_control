# Get the path to the package
import rospkg
import sys
import os
rospack = rospkg.RosPack()
package_path = rospack.get_path('pybullet')

# Add the path to the package to the Python path
sys.path.append(os.path.join(package_path, 'scripts'))

import pybullet as p
import pybullet_data
import numpy as np
from config import RobotConfig, SimulationConfig

class PyBulletEnv:
    def __init__(self, robot_config: RobotConfig, simulation_config: SimulationConfig, use_gui=True):
        self.use_gui = use_gui

        self.robot_config = robot_config
        self.sim_config = simulation_config
        
        # Initialize PyBullet
        if self.use_gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
        
        # Set simulation parameters
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Load the environment and robot
        self.load_environment()
        self.load_robot()

        self.previous_linear_velocity = np.zeros(3)
    
    def load_environment(self):
        p.resetSimulation()
        if self.use_gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=70, cameraPitch=-40,
                                    cameraTargetPosition=[0,0,0.8])
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(fixedTimeStep=self.sim_config.simulation_time_step)
        self.plane_id = p.loadURDF("plane.urdf",useFixedBase=True)
        p.changeDynamics(bodyUniqueId=self.plane_id,linkIndex=-1,lateralFriction=self.sim_config.ground_lateral_friction,
                        rollingFriction=self.sim_config.ground_rolling_friction,spinningFriction=self.sim_config.ground_spinning_friction,linearDamping=0,angularDamping=0,
                        mass=0,localInertiaDiagonal=[0, 0, 0],restitution=self.sim_config.ground_restitution)
        
    def load_robot(self):
        initial_body_quaternion = p.getQuaternionFromEuler(self.robot_config.initial_body_rpy)
        self.robot_id = p.loadURDF(self.robot_config.urdf_path, self.robot_config.initial_body_pos, initial_body_quaternion,
                            flags = p.URDF_MERGE_FIXED_LINKS|p.URDF_USE_SELF_COLLISION|p.URDF_USE_INERTIA_FROM_FILE)
        self.num_joints = p.getNumJoints(self.robot_id)
        for i in range(self.num_joints):
            p.resetJointState(self.robot_id,i,self.robot_config.initial_joint_pos[i])
        # disable default controller
        p.setJointMotorControlArray(self.robot_id, range(self.num_joints), controlMode=p.VELOCITY_CONTROL, forces=[0]*12)
    
    def load_soccer_ball(self):
        self.ball_id = p.loadURDF("soccerball.urdf",[0.5,0,1], globalScaling=0.21)
        p.changeDynamics(self.ball_id,-1, mass=0.25, linearDamping=0, angularDamping=0, rollingFriction=0.001, spinningFriction=0.001, restitution=0.5)
        p.changeVisualShape(self.ball_id,-1,rgbaColor=[0.8,0.8,0.8,1])

    # control
    def set_joint_torques(self, joint_torques):
        p.setJointMotorControlArray(self.robot_id, range(self.num_joints), controlMode=p.TORQUE_CONTROL, forces=joint_torques)
            # p.setJointMotorControl2(self.robot_id, joint_index, controlMode=p.POSITION_CONTROL, force=10.0,
            #                         targetPosition = torque,maxVelocity=4)
    
    # observation
    def get_joint_states(self):
        states = p.getJointStates(self.robot_id, [x for x in range(self.num_joints)])
        qpos = [s[0] for s in states]
        qvel = [s[1] for s in states]
        # react = [s[2] for s in states]
        torq = [s[3] for s in states]
        return qpos, qvel, torq
    
    def get_imu_observation(self):
         # Get base position and orientation
        pos, ori = p.getBasePositionAndOrientation(self.robot_id)
        quaternion = [ori[3], ori[0], ori[1], ori[2]]
        # Get base velocity (linear and angular)
        linear_vel, angular_vel_world = p.getBaseVelocity(self.robot_id)
        
        # Compute linear acceleration
        linear_velocity = np.array(linear_vel)
        linear_acc = (linear_velocity - self.previous_linear_velocity) / self.sim_config.simulation_time_step
        self.previous_linear_velocity = linear_velocity
        
        # Convert angular velocity to local frame
        orn_matrix = p.getMatrixFromQuaternion(ori)
        orn_matrix = np.array(orn_matrix).reshape(3, 3)
        angular_vel_local = np.dot(orn_matrix.T, np.array(angular_vel_world))

        return quaternion, angular_vel_local, linear_acc


    def step_simulation(self):
        p.stepSimulation()
    
    def disconnect(self):
        p.disconnect()


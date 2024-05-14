from math import pi
# Get the path to the package
import rospkg
import sys
import os
rospack = rospkg.RosPack()
package_path = rospack.get_path('pybullet')

class RobotConfig(object):
    # gr1t1 config
    urdf_path = os.path.join(package_path, 'assets/GR1T1/urdf/GR1T1_fixed_upper_body.urdf')
    lower_joints = [-0.4, -2.0, -0.2, -0.1, -pi / 2.5, -0.5,  # left leg
                    0, 0, -pi, -1.6, 0, -pi, -1.6, -2.5,  # head & arm
                    -0.4, -1.8, -0.2, -1.9, -pi / 2.5, -0.5,  # right leg
                    ]
    upper_joints = [0.4, 1.8, 0.2, 1.9, pi / 2.5, 0.5,
                    0, 0, pi, 1.6, 2.5, pi, 1.6, 0,
                    0.4, 2.0, 0.2, 0.1, pi / 2.5, 0.5,
                    ]
    initial_joint_pos = [0, 0, -0.46, 0.9, -0.44, 0, 
                         0, 0, -0.46, 0.9, -0.44, 0]
    initial_body_pos = [0, 0, 0.88]
    initial_body_rpy = [0, 0, 0]

class SimulationConfig(object):
    simulation_time_step = 0.002 # simulation frequency 500 Hz

    # ground
    ground_lateral_friction = 1.0
    ground_spinning_friction = 0.1
    ground_rolling_friction = 0.001
    ground_restitution = 0.9

#!/usr/bin/env python
# Get the path to the package
import rospkg
import sys
import os
import select
import threading
rospack = rospkg.RosPack()
package_path = rospack.get_path('pybullet')

# Add the path to the package to the Python path
sys.path.append(os.path.join(package_path, 'scripts'))

import pybullet as p
import time
import numpy as np
from config import RobotConfig, SimulationConfig
from pybullet_env import PyBulletEnv
from pybullet_lcm import Lcm


if __name__ == '__main__':
    robot_config = RobotConfig()
    simulation_config = SimulationConfig()
    env = PyBulletEnv(robot_config, simulation_config)
    lcm = Lcm()
    # previous_cmd_ff_tau = [0]*12
    # previous_cmd_joint_pos = robot_config.initial_joint_pos
    # previous_cmd_joint_vel = [0]*12
    # previous_cmd_kp = [20]*12
    # previous_cmd_kd = [10]*12

    # p.setRealTimeSimulation(1)

    pause = True

    def keyboard_listener():
        time.sleep(4)
        global pause
        pause = False
    
    threading.Thread(target=keyboard_listener, daemon=True).start()

    while(True):
        if pause:
            lcm.set_send([1, 0, 0, 0], [0]*3, [0, 0, 9.81], robot_config.initial_joint_pos, [0]*12, [0]*12)
            lcm.send()
            lcm.lcm_.handle()
            # cmd = lcm.get_recv()
            # print(cmd.ff_tau)

            time.sleep(0.0005)
        else:
            break

    last_time = time.perf_counter()
    try:
        while(True):
            current_time = time.perf_counter()
            lcm.lcm_.handle()
            if time.perf_counter() - last_time > simulation_config.simulation_time_step:
                last_time = current_time
                joint_pos, joint_vel, applied_torq = env.get_joint_states()
                ori, angular_vel_local, linear_acc = env.get_imu_observation()
                # print(joint_pos, joint_vel, applied_torq)
                # time.sleep(100)
                lcm.set_send(ori, angular_vel_local, linear_acc, joint_pos, joint_vel, applied_torq)
                lcm.send()
                cmd = lcm.get_recv()
                # torques = [cmd.ff_tau[i] + cmd.kp[i] * (cmd.joint_pos[i] - joint_pos[i]) + cmd.kd[i] * (cmd.joint_vel[i] - joint_vel[i]) for i in range(12)]
                torques = cmd.ff_tau
                env.set_joint_torques(torques)
                env.step_simulation()

    except KeyboardInterrupt:
        env.disconnect()

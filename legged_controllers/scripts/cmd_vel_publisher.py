#!/usr/bin/env python
import numpy as np

import rospy
from ocs2_msgs.msg import mpc_observation
from geometry_msgs.msg import Twist


class cmdVelController:
    def __init__(self) -> None:
        ball_position = np.array([3.0, -0.1])
        goal_position = np.array([6.0, 0.0])

        align_target_pose = np.zeros(3)
        kick_target_pose = np.zeros(3)

        align_target_pose[:2] = ball_position - 0.5*(goal_position - ball_position)/np.linalg.norm(goal_position - ball_position)
        align_target_pose[2] = np.arctan2((goal_position - ball_position)[1], (goal_position - ball_position)[0])

        kick_target_pose[:2] = ball_position + 0.5*(goal_position - ball_position)/np.linalg.norm(goal_position - ball_position)
        kick_target_pose[2] = np.arctan2((goal_position - ball_position)[1], (goal_position - ball_position)[0])

        self.ball_position = ball_position
        self.goal_position = goal_position
        self.align_target_pose = align_target_pose
        self.kick_target_pose = kick_target_pose

        self.test_target_pose = np.array([1.5, 0.5, 0.5])

        # Initialize the ROS node
        rospy.init_node('cmd_vel_publisher', anonymous=True)

        # Create a subscriber to the specified topic
        rospy.Subscriber("legged_robot_mpc_observation", mpc_observation, self.callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def callback(self, data: mpc_observation):
        current_state = data.state.value
        current_pose = np.zeros(3)
        current_pose[0] = current_state[6]
        current_pose[1] = current_state[7]
        current_pose[2] = current_state[9]

        kp = np.array([0.05, 0.5, 0.5])

        cmd_vel = kp * (self.test_target_pose - current_pose)
        cmd_vel = cmd_vel.clip(min=[-0.3, -0.1, -0.2], max=[0.2, 0.1, 0.2])

        vel_msg = Twist()
        vel_msg.linear.x = cmd_vel[0]
        vel_msg.linear.y = cmd_vel[1]
        vel_msg.angular.z = cmd_vel[2]

        self.cmd_vel_pub.publish(vel_msg)
        print("\033[H\033[J", end="")
        print(f"current_pose: {current_pose}\ncmd_vel: {cmd_vel}")

    def spin(self):
        # Keep the script alive until the node is shutdown
        rospy.spin()


if __name__ == '__main__':
    controller = cmdVelController()
    controller.spin()
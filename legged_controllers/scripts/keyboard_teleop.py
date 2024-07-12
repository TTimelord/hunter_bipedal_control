#!/usr/bin/env python3

import pynput
from pynput.keyboard import Key

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class Teleop:
    def __init__(self):
        self._is_running = True
        self._listener = pynput.keyboard.Listener(
            on_press=self._on_press,
        )
        self._listener.start()
        self._command = [0.0, 0.0, 0.0]
        self._request_kick = "none"
        self._request_stance = False
        self._request_walk = False
        
        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._stance_pub = rospy.Publisher("/switch_to_stance", Int32, queue_size=1)
        self._walk_pub = rospy.Publisher("/switch_to_walk", Int32, queue_size=1)
        self._kick_pub = rospy.Publisher("/request_kick", Int32, queue_size=1)

    def publish(self):
        msg = Twist()
        msg.linear.x = self._command[0]
        msg.linear.y = self._command[1]
        msg.angular.z = self._command[2]
        self._cmd_pub.publish(msg)

        if self._request_stance:
            stance_msg = Int32()
            self._stance_pub.publish(stance_msg)
        
        if self._request_walk:
            walk_msg = Int32()
            self._walk_pub.publish(walk_msg)        

        if self._request_kick == "left":
            kick_msg = Int32()
            kick_msg.data = 0
            self._kick_pub.publish(kick_msg)

        if self._request_kick == "right":
            kick_msg = Int32()
            kick_msg.data = 1
            self._kick_pub.publish(kick_msg)

        self._request_kick = "none"
        self._request_stance = False
        self._request_walk = False

    def is_running(self):
        return self._is_running

    def _on_press(self, key):
        if key.char == ('w'):
            self._command[0] += 0.1
        elif key.char == ('s'):
            self._command[0] -= 0.1
        elif key.char == ('d'):
            self._command[1] += 0.05
        elif key.char == ('a'):
            self._command[1] -= 0.05
        elif key.char == ('q'):
            self._command[2] += 0.1
        elif key.char == ('e'):
            self._command[2] -= 0.1
        elif key.char == ('j'):
            self._request_stance = True
        elif key.char == ('k'):
            self._request_walk = True
        elif key.char == ('u'):
            self._request_kick = "left"
        elif key.char == ('i'):
            self._request_kick = "right"

def main():
    rospy.init_node("better_teleop")
    teleop = Teleop()

    rate = rospy.Rate(10)
    while teleop.is_running():
        teleop.publish()
        print("\033[H\033[J", end="")
        print(f"cmd_vel:\n {teleop._command[0]}\n{teleop._command[1]}\n{teleop._command[2]}")
        rate.sleep()

    return 0


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
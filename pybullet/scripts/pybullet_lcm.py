import threading
from time import time
from lcm import LCM
import rospkg
import os
import sys
# Get the path to the package
rospack = rospkg.RosPack()
package_path = rospack.get_path('lcm_msg')

# Add the path to the package to the Python path
sys.path.append(os.path.join(package_path, 'include/lcm_msg'))

from low_cmd_t import low_cmd_t
from low_state_t import low_state_t

class Lcm:
    def __init__(self):
        self.recv_mutex = threading.Lock()
        self.send_mutex = threading.Lock()
        self.lcm_ = LCM()
        self.lcm_.subscribe("LOWCMD", self.handle_low_cmd)
        self.recv_cmd = low_cmd_t()
        self.send_state = low_state_t()
        self.last_timestamp = 0
        self.new_msg = False
    
    def handle_low_cmd(self, channel, data):
        msg = low_cmd_t.decode(data)
        with self.recv_mutex:
            if msg.timestamp > self.last_timestamp:
                self.recv_cmd = msg
                self.last_timestamp = msg.timestamp
                self.new_msg = True
    
    def get_recv(self):
        with self.recv_mutex:
            # if self.new_msg:
            #     cmd = self.recv_cmd
            #     self.new_msg = False
            #     return cmd
            # else:
            #     return None
            return self.recv_cmd
    
    def set_send(self, ori, agular_vel, acc, joint_pos, joint_vel, joint_torque):
        with self.send_mutex:
            current_time_point = int(time() * 1e9)  # Convert current time to nanoseconds
            self.send_state.timestamp = current_time_point

            self.send_state.quaternion = ori
            self.send_state.accelerometer = acc
            self.send_state.gyroscope = agular_vel

            self.send_state.joint_pos = joint_pos
            self.send_state.joint_vel = joint_vel
            self.send_state.joint_torque = joint_torque

    def send(self):
        with self.send_mutex:
            self.lcm_.publish("LOWSTATE", self.send_state.encode())

# Example usage
if __name__ == "__main__":
    mujoco_lcm = Lcm()
    # Assuming you have a method to get mjData
    d = get_mjData()  # This function should provide the data similar to the C++ version
    mujoco_lcm.set_send(d)
    mujoco_lcm.send()

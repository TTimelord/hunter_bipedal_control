# MPC environment setup (on GR1's nuc)
1. Install ROS according to http://wiki.ros.org/noetic/Installation/Ubuntu
    1. if "gpg: no valid OpenPGP data found": https://answers.ros.org/question/157766/unable-to-locate-package-no-valid-openpgp-data/
    2. install the full version of ROS `sudo apt install ros-noetic-desktop-full`
2. Install Pinocchio according to https://stack-of-tasks.github.io/pinocchio/download.html
3. Install rapid jason (required by FSE):
```bash
git clone https://github.com/Tencent/rapidjson.git
cd rapidjson/
mkdir build
cd build
cmake ..
make
sudo make install
```

4. Download and compile mpc package
```bash
sudo apt install git libglpk-dev 
sudo apt install catkin # it's possible that there are errors
sudo apt install ros-noetic-pybind11-catkin python3-catkin-tools
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev

mkdir -p <directory_to_ws>/mpc_ws/src
cd <directory_to_ws>/mpc_ws/src
git clone https://github.com/TTimelord/hunter_bipedal_control.git
git clone https://github.com/leggedrobotics/ocs2.git
git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git

cd ..
catkin init
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo

catkin build legged_controllers gr1t2_description gr1_hardware_interface 
```

5. set thread priority according to https://leggedrobotics.github.io/ocs2/faq.html

# For simulation
1. Install lcm
http://lcm-proj.github.io/lcm/content/build-instructions.html#build-instructions
2. Build mujoco and legged mujoco
```
catkin build legged_mujoco mujoco
```

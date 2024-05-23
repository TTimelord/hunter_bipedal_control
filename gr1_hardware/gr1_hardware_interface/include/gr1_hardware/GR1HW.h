#pragma once
#include <iostream>
#include <legged_hw/LeggedHW.h>
#include <ros/ros.h>
#include <ch108IMU/ch108IMU.h>
#include <parallel_ankle/parallel_ankle.h>
#include <Fsa.h>

#define TOTAL_JOINT_NUM 12
#define JOINT_NUM 6 // number of joints per leg

// #include <dynamic_reconfigure/server.h>

namespace legged {

// "RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT" 
// fr fl rr rl
// const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct MotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct ImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};


class GR1HW : public LeggedHW
{
  public:
  GR1HW() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

  ~GR1HW();

private:
  bool setupJoints();

  bool setupImu();

  // bool setupContactSensor(ros::NodeHandle& nh);

  // void ConfigCallback(legged_mujoco_config::touchConfig &config)
  // {
  //     contactThreshold_ = config.contactThreshold;
  // }

  // dynamic_reconfigure::Server<legged_mujoco_config::touchConfig> server;
  // dynamic_reconfigure::Server<legged_mujoco_config::touchConfig>::CallbackType f;

  MotorData jointData_[12]{};  // NOLINT(modernize-avoid-c-arrays)
  ImuData imuData_{};
  // bool contactState_[4]{};  // NOLINT(modernize-avoid-c-arrays)

// 
  // int contactThreshold_{};

  ch108::ch108IMU imu;
  parallel_ankle funS2P;

  std::vector<FSA_CONNECT::FSA> fsa_list = std::vector<FSA_CONNECT::FSA>(TOTAL_JOINT_NUM);
  std::vector<std::string> ip_list = {"", ""};

  double torque_to_current(int index, double torque);

  std::vector<double> ratio = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 
                                1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<double> scale = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 
                                1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<double> dir = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 
                              1.0, 1.0, 1.0, 1.0, 1.0, 1.0};                                
};


}



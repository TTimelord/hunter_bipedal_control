#include "gr1_hardware/GR1HW.h"

namespace legged {

bool GR1HW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  setupJoints();
  setupImu();
  // setupContactSensor(robot_hw_nh);

  // f = boost::bind(&GR1HW::ConfigCallback, this, _1);
  // server.setCallback(f);

  int ret = 0;
  for (int i = 0; i < TOTAL_JOINT_NUM; i++) {
      fsa_list[i].init(ip_list[i]);
      ret = fsa_list[i].Enable();
      if (ret < 0) {
          std::cout << "wrong" << std::endl;
          exit(0);
      }
  }

  for (int i = 0; i < TOTAL_JOINT_NUM; i++) {
      fsa_list[i].EnablePosControl();
  }

  return true;
}    


void GR1HW::read(const ros::Time& time, const ros::Duration& /*period*/) {
  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    fsa_list[i].GetPVC(jointData_[i].pos_, jointData_[i].vel_, jointData_[i].tau_); // TODO: current to tau conversion required
  }

  Eigen::Quaterniond quaternion = Eigen::AngleAxisd(imu.imudata(0), Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(imu.imudata(1), Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(imu.imudata(2), Eigen::Vector3d::UnitX());
  
  imuData_.ori_[0] = quaternion.coeffs()(0);
  imuData_.ori_[1] = quaternion.coeffs()(1);
  imuData_.ori_[2] = quaternion.coeffs()(2);
  imuData_.ori_[3] = quaternion.coeffs()(3);
  imuData_.angularVel_[0] = imu.imudata(3);
  imuData_.angularVel_[1] = imu.imudata(4);
  imuData_.angularVel_[2] = imu.imudata(5);
  imuData_.linearAcc_[0] = imu.imudata(6);
  imuData_.linearAcc_[1] = imu.imudata(7);
  imuData_.linearAcc_[2] = imu.imudata(8);

  // for (int i = 0; i < 4; ++i) {
  //   contactState_[i] = lowState_.foot_force[i] > contactThreshold_;
  // }

}

void GR1HW::write(const ros::Time& time, const ros::Duration& /*period*/) {

  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    fsa_list[i].SetPosition(jointData_[i].posDes_, jointData_[i].velDes_, torque_to_current(i, jointData_[i].ff_));
    // lowCmd_.joint_pos[i] = static_cast<double>(jointData_[i].posDes_);
    // lowCmd_.joint_vel[i] = static_cast<double>(jointData_[i].velDes_);
    // lowCmd_.kp[i] = static_cast<double>(jointData_[i].kp_);
    // lowCmd_.kd[i] = static_cast<double>(jointData_[i].kd_);
    // lowCmd_.ff_tau[i] = static_cast<double>(jointData_[i].ff_);
  }
}

double GR1HW::torque_to_current(int index, double torque){
  double current = torque * dir[index]/(ratio[index]*scale[index]);
  return current;
}

bool GR1HW::setupJoints() {
  for (const auto& joint : urdfModel_->joints_)
  {
    int leg_index, joint_index;
    if (joint.first.find("l_") != std::string::npos)
    {
      leg_index = 0;
    }
    else if (joint.first.find("r_") != std::string::npos)
    {
      leg_index = 1;
    }
    else
      continue;
    if (joint.first.find("hip_roll") != std::string::npos)
      joint_index = 0;
    else if (joint.first.find("hip_yaw") != std::string::npos)
      joint_index = 1;
    else if (joint.first.find("hip_pitch") != std::string::npos)
      joint_index = 2;
    else if (joint.first.find("knee_pitch") != std::string::npos)
      joint_index = 3;
    else if (joint.first.find("ankle_pitch") != std::string::npos)
      joint_index = 4;
    else if (joint.first.find("ankle_roll") != std::string::npos)
      joint_index = 5;
    else
      continue;

    int index = leg_index * JOINT_NUM + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_,
                                                           &jointData_[index].velDes_, &jointData_[index].kp_,
                                                           &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool GR1HW::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("imu_link", "imu_link", imuData_.ori_, imuData_.oriCov_,
                                                                         imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                         imuData_.linearAccCov_));
  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  imu.initIMU();

  return true;
}

// bool GR1HW::setupContactSensor(ros::NodeHandle& nh) {
//   nh.getParam("contact_threshold", contactThreshold_);
//   for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
//     contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
//   }
//   return true;
// }


GR1HW:: ~GR1HW(){
}

}


#include "gr1_hardware/GR1HW.h"

namespace legged {

bool GR1HW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  rapidjson::Document msg_json;
  char ser_msg[1024] = {0};

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

  for (int i = 0; i < TOTAL_JOINT_NUM; i++) {
    fse.demo_get_measured(ae_ip_list[i], NULL, ser_msg);
    fsa_list[i].GetPVC(read_joint_pos[i], read_joint_vel[i], read_joint_torq[i]); 
    if (msg_json.Parse(ser_msg).HasParseError())
    {
        std::cout<<"fi_decode()"<<i<<"failed\n";
        return 0;
    }
    double ae_current = msg_json["radian"].GetDouble();
    pos_offset[i] = ae_current - absolute_pos_zero[i] - read_joint_pos[i]*PI/180;
  }
  return true;
}    


void GR1HW::read(const ros::Time& time, const ros::Duration& /*period*/) {
  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    fsa_list[i].GetPVC(read_joint_pos[i], read_joint_vel[i], read_joint_torq[i]); // TODO: current to tau conversion required
    read_joint_pos[i] = dir[i] * (PI / 180 * read_joint_pos[i] + pos_offset[i]);
    read_joint_vel[i] = dir[i] * PI / 180 * read_joint_vel[i];
    read_joint_torq[i] = current_to_torque(i, read_joint_torq[i]);
  }

  Eigen::VectorXd motorPos = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd motorVel = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd motorTorq = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd anklePose = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd ankleVel = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd ankleTorq = Eigen::VectorXd::Zero(4);

  motorPos << read_joint_pos[4], read_joint_pos[5], read_joint_pos[10], read_joint_pos[11];
  motorVel << read_joint_vel[4], read_joint_vel[5], read_joint_vel[10], read_joint_vel[11];
  motorTorq << read_joint_torq[4], read_joint_torq[5], read_joint_torq[10], read_joint_torq[11];

  funS2P.setEst(motorPos, motorVel, motorTorq);
  funS2P.calcFK();
  funS2P.getAnkleState(anklePose, ankleVel, ankleTorq);

  read_joint_pos[4] = anklePose[0];
  read_joint_pos[5] = anklePose[1];
  read_joint_pos[10] = anklePose[2];
  read_joint_pos[11] = anklePose[3];

  read_joint_vel[4] = ankleVel[0];
  read_joint_vel[5] = ankleVel[1];
  read_joint_vel[10] = ankleVel[2];
  read_joint_vel[11] = ankleVel[3];

  read_joint_torq[4] = ankleTorq[0];
  read_joint_torq[5] = ankleTorq[1];
  read_joint_torq[10] = ankleTorq[2];
  read_joint_torq[11] = ankleTorq[3];

  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    jointData_[i].pos_ = read_joint_pos[i];
    jointData_[i].vel_ = read_joint_vel[i];
    jointData_[i].tau_ = read_joint_torq[i];
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
    write_joint_pos[i] = jointData_[i].posDes_;
    write_joint_vel[i] = jointData_[i].velDes_;
    write_joint_torq[i] = jointData_[i].ff_;
  }

  Eigen::VectorXd motorPos = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd motorVel = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd motorTorq = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd anklePose = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd ankleVel = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd ankleTorq = Eigen::VectorXd::Zero(4);

  anklePose << write_joint_pos[4], write_joint_pos[5], write_joint_pos[10], write_joint_pos[11];
  ankleVel << write_joint_vel[4], write_joint_vel[5], write_joint_vel[10], write_joint_vel[11];
  ankleTorq << write_joint_torq[4], write_joint_torq[5], write_joint_torq[10], write_joint_torq[11];

  funS2P.setRef(anklePose, ankleVel, ankleTorq);
  funS2P.calcIK();
  funS2P.getMotorCmd(motorPos, motorVel, motorTorq);

  write_joint_pos[4] = motorPos[0];
  write_joint_pos[5] = motorPos[1];
  write_joint_pos[10] = motorPos[2];
  write_joint_pos[11] = motorPos[3];

  write_joint_vel[4] = motorVel[0];
  write_joint_vel[5] = motorVel[1];
  write_joint_vel[10] = motorVel[2];
  write_joint_vel[11] = motorVel[3];

  write_joint_torq[4] = motorTorq[0];
  write_joint_torq[5] = motorTorq[1];
  write_joint_torq[10] = motorTorq[2];
  write_joint_torq[11] = motorTorq[3]; 

  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    write_joint_pos[i] = (write_joint_pos[i] - pos_offset[i]) * 180/PI;
    write_joint_vel[i] = write_joint_vel[i] * 180/PI;
    fsa_list[i].SetPosition(dir[i] * write_joint_pos[i], dir[i] * write_joint_vel[i], torque_to_current(i, write_joint_torq[i]));
  }
}

double GR1HW::torque_to_current(int index, double torque){
  double current = torque * dir[index]/(ratio[index]*scale[index]);
  return current;
}

double GR1HW::current_to_torque(int index, double current){
  double torque = current * (ratio[index]*scale[index]) * dir[index];
  return torque;
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


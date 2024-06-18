#include "gr1_hardware/GR1HW.h"
#include <string>
#include <nlohmann/json.hpp>  // Include the JSON library header
#include <fstream>            // For file operations
#include <ocs2_core/misc/LoadData.h>
#include <chrono>
#include <thread>
#include <FsaConfig.h>

// #define ESTIMATION_ONLY
// #define TIMER

namespace legged {

bool GR1HW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  root_nh.getParam("/referenceFile", referenceFile);
  root_nh.getParam("/motorlistFile", motorlistFile);

  // add joint states publisher
  debug_joint_pub = root_nh.advertise<sensor_msgs::JointState>("debug_joint_states", 1);
  joint_state_gr1.name.resize(TOTAL_JOINT_NUM);
  joint_state_gr1.effort.resize(TOTAL_JOINT_NUM);
  joint_state_gr1.velocity.resize(TOTAL_JOINT_NUM);
  joint_state_gr1.position.resize(TOTAL_JOINT_NUM);
  for (int i = 0; i < TOTAL_JOINT_NUM; i++) {
      joint_state_gr1.name[i] = ip_list[i];
  }

  default_joint_pos.setZero(TOTAL_JOINT_NUM+3); // leg + waist
  ocs2::loadData::loadEigenMatrix(referenceFile, "defaultJointState", default_joint_pos);

  setupJoints();
  setupImu();
  // setupContactSensor(robot_hw_nh);

  // f = boost::bind(&GR1HW::ConfigCallback, this, _1);
  // server.setCallback(f);

  // ================== set PID params ======================
  ifstream ifs(motorlistFile);
  // Parse the JSON data
  nlohmann::json j;
  ifs >> j;

  // Populate the vectors according to the order in ip_list
  for (const string& ip : ip_list) {
      if (j.find(ip) != j.end()) {
          ratio.push_back(j[ip]["motor_gear_ratio"]);
          scale.push_back(j[ip]["c_t_scale"]);
          absolute_pos_zero.push_back(j[ip]["absolute_pos_zero"]);
          absolute_pos_dir.push_back(j[ip]["absolute_pos_dir"]);
          absolute_pos_ratio.push_back(j[ip]["absolute_pos_gear_ratio"]);
          motor_dir.push_back(j[ip]["motorDir"]);
          pos_gain.push_back(j[ip]["controlConfig"]["pos_gain"]);
          vel_gain.push_back(j[ip]["controlConfig"]["vel_gain"]);
          vel_integrator_gain.push_back(j[ip]["controlConfig"]["vel_integrator_gain"]);
      } else {
          // If the IP is not found in the JSON
          ROS_ERROR("An error occurred when reading motorlist.json.");
          exit(1);
      }
  }

  std::vector<double> arm_head_pos_gain; 
  std::vector<double> arm_head_vel_gain;
  // get pid for arm and head
  for (const string& ip : arm_and_head_ip_list) {
      if (j.find(ip) != j.end()) {
          arm_head_pos_gain.push_back(j[ip]["controlConfig"]["pos_gain"]);
          arm_head_vel_gain.push_back(j[ip]["controlConfig"]["vel_gain"]);
      } else {
          // If the IP is not found in the JSON
          ROS_ERROR("An error occurred when reading motorlist.json.");
          exit(1);
      }
  }

  FSA_CONNECT::FSAConfig::FSAPIDParams pid_params;
  FSA_CONNECT::FSAConfig::FSAPIDParams get_pid_params;

  int ret = 0;
  for (int i = 0; i < TOTAL_JOINT_NUM + 3; i++) { //consider legs + waist
      fsa_list[i].init(ip_list[i]);
      pid_params.control_position_kp = pos_gain[i];
      pid_params.control_velocity_kp = vel_gain[i];
      std::cout<<"write kp: "<<pid_params.control_position_kp << "kd:" << pid_params.control_velocity_kp << std::endl;
      fsa_list[i].SetPIDParams(pid_params);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      fsa_list[i].GetPVC(read_joint_pos[i], read_joint_vel[i], read_joint_torq[i]); // read PVC for the first time to get rid of errors
      fsa_list[i].GetPIDParams(get_pid_params);
      std::cout<<"read kp: "<<get_pid_params.control_position_kp << "kd:" << get_pid_params.control_velocity_kp << std::endl;
      last_cmd_pos[i] = read_joint_pos[i];
      last_cmd_cur[i] = 0;
      last_cmd_vel[i] = 0;
  }

  for (int i = 0; i < arm_and_head_ip_list.size(); i++) { // arm and head
      arm_and_head_fsa_list[i].init(arm_and_head_ip_list[i]);
      pid_params.control_position_kp = arm_head_pos_gain[i];
      pid_params.control_velocity_kp = arm_head_vel_gain[i];
      std::cout<<"write kp: "<<pid_params.control_position_kp << "kd:" << pid_params.control_velocity_kp << std::endl;
      arm_and_head_fsa_list[i].SetPIDParams(pid_params);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      arm_and_head_fsa_list[i].GetPIDParams(get_pid_params);
      std::cout<<"read kp: "<<get_pid_params.control_position_kp << "kd:" << get_pid_params.control_velocity_kp << std::endl;
  }
  // ================== END set PID params ======================


  if(!calculate_offset()){
    ROS_ERROR("calculate_offset() failed");
    exit(1);
  }

  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    jointData_[i].posDes_ = default_joint_pos[i];  // initialize jointData_[i].posDes_ to avoid jumping
  }

  #ifndef ESTIMATION_ONLY
  for (int i = 0; i < TOTAL_JOINT_NUM+3; i++) {
      ret = fsa_list[i].Enable();
      if (ret < 0) {
          std::cout << "wrong" << std::endl;
          exit(0);
      }
  }

  for (int i = 0; i < TOTAL_JOINT_NUM+3; i++) {
      fsa_list[i].EnablePosControl();
  }

  for (int i = 0; i<arm_and_head_ip_list.size(); i++) {
    arm_and_head_fsa_list[i].Enable();
    arm_and_head_fsa_list[i].EnablePosControl();
  }

  std::this_thread::sleep_for(std::chrono::seconds(3));

  if(!go_to_default_pos()){
    ROS_ERROR("go_to_default_pos() failed");
    exit(1);
  }
  #endif

  // exit(0);

  return true;
}    

bool GR1HW::calculate_offset(){
  rapidjson::Document msg_json;
  char ser_msg[1024] = {0};
  for (int i = 0; i < TOTAL_JOINT_NUM + 3; i++) {   // consider waist joints;
    fse.demo_get_measured(ae_ip_list[i], NULL, ser_msg);
    fsa_list[i].GetPVC(read_joint_pos[i], read_joint_vel[i], read_joint_torq[i]); 
    if (msg_json.Parse(ser_msg).HasParseError())
    {
        std::cout<<"fi_decode()"<<i<<"failed\n";
        return 0;
    }
    double ae_current = msg_json["radian"].GetDouble();
    pos_offset[i] = ae_current - absolute_pos_zero[i];
    // std::cout<<i<<"======="<<std::endl;
    // std::cout<<ae_current<<std::endl;
    // std::cout<<absolute_pos_zero[i]<<std::endl;

    while(pos_offset[i]<-PI){
      pos_offset[i] += 2*PI;
    }
    while(pos_offset[i]>PI){
      pos_offset[i] -= 2*PI;
    }
    // std::cout<<pos_offset[i]<<std::endl;
    pos_offset[i] = absolute_pos_dir[i]*pos_offset[i]/absolute_pos_ratio[i] - motor_dir[i]*read_joint_pos[i]*PI/180;
  }
  return true;
}

bool GR1HW::go_to_default_pos(){    
  const double frequency = 500; // Hz
  const double period = 1.0 / frequency;
  const double duration = 5;
  using clock = std::chrono::steady_clock;
  using milliseconds = std::chrono::milliseconds;

  auto start_time = clock::now();
  auto next_time = clock::now() + milliseconds(static_cast<int>(1000 / frequency));

  for (int i = 0; i < TOTAL_JOINT_NUM+3; ++i) {
    fsa_list[i].GetPVC(read_joint_pos[i], read_joint_vel[i], read_joint_torq[i]);
    write_joint_pos[i] = default_joint_pos[i];
  }

  serial_to_parallel();
  std::vector<double> velocity(write_joint_pos.size());
  std::vector<double> delta_pos(write_joint_pos.size());
  std::vector<double> target_pos(write_joint_pos.size());
  for (int i = 0; i < TOTAL_JOINT_NUM+3; ++i) {
    target_pos[i] = (write_joint_pos[i] - pos_offset[i]) * 180/PI * motor_dir[i];
    velocity[i] = (target_pos[i] - read_joint_pos[i])/duration;
    delta_pos[i] = (target_pos[i] - read_joint_pos[i])/(frequency*duration);
    write_joint_pos[i] = read_joint_pos[i];
  }

  while (true) {
      auto now = clock::now();
      if(now > start_time + milliseconds(int(1000*duration))){
        break;
      }
      if (now < next_time) {
          now = clock::now();
          continue;
      }

      // Your loop code here
      for (int i = 0; i < TOTAL_JOINT_NUM+3; ++i) {
        fsa_list[i].GetPVC(read_joint_pos[i], read_joint_vel[i], read_joint_torq[i]);
      }

      // serial_to_parallel();

      for (int i = 0; i < TOTAL_JOINT_NUM+3; ++i) {
        // write_joint_pos[i] = (write_joint_pos[i] - pos_offset[i]) * 180/PI * motor_dir[i];

        write_joint_pos[i] += delta_pos[i];
        // std::cout<<i<<" "<<write_joint_pos[i]<<" "<<velocity[i]<<std::endl;
        // std::cout<<read_joint_pos[i]<<std::endl;
        last_cmd_pos[i] = write_joint_pos[i];
        fsa_list[i].SetPosition(write_joint_pos[i], 0, 0);
      }

      // Update the time for the next iteration
      next_time += milliseconds(static_cast<int>(period * 1000));
      // std::cout<<std::chrono::duration_cast<milliseconds>(next_time.time_since_epoch()).count()<<std::endl;
  }

  return true;
}

void GR1HW::read(const ros::Time& time, const ros::Duration& /*period*/) {
  #ifdef TIMER
  auto read_start_time = std::chrono::steady_clock::now();
  #endif

  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    fsa_list[i].GetPVC(current_motor_pos[i], current_motor_vel[i], current_motor_cur[i]); // TODO: current to tau conversion required
    read_joint_pos[i] = motor_dir[i] * (PI / 180 * current_motor_pos[i]) + pos_offset[i];
    read_joint_vel[i] = motor_dir[i] * PI / 180 * current_motor_vel[i];

    // joint_state_gr1.header.stamp = ros::Time::now();
    // joint_state_gr1.position[i] = current_motor_pos[i];
    // joint_state_gr1.velocity[i] = current_motor_vel[i];
    // joint_state_gr1.effort[i] = current_motor_cur[i];

    std::cerr<<"read motor "<<i<<" pos: "<< current_motor_pos[i] << "vel: "<< current_motor_vel[i] << "torq:" << read_joint_torq[i] <<"\n";
  }
  // debug_joint_pub.publish(joint_state_gr1);

  parallel_to_serial();

  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    jointData_[i].pos_ = read_joint_pos[i];
    jointData_[i].vel_ = read_joint_vel[i];
    jointData_[i].tau_ = read_joint_torq[i];
    // std::cout<<"read joint "<<i<<" pos: "<< read_joint_pos[i] << "vel: "<< read_joint_vel[i] << "torq:" << read_joint_torq[i] <<"\n";
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

  // joint_state_gr1.header.stamp = ros::Time::now();
  // joint_state_gr1.position[0] = imuData_.angularVel_[0];
  // joint_state_gr1.position[1] =  imuData_.angularVel_[1];
  // joint_state_gr1.position[2] =  imuData_.angularVel_[2];
  // debug_joint_pub.publish(joint_state_gr1);

  // std::cout<<imu.imudata(0)<<" "<<imu.imudata(1)<<" "<<imu.imudata(2)<<std::endl;
  // std::cout<<imuData_.ori_[0]<<" "<<imuData_.ori_[1]<<" "<<imuData_.ori_[2]<<" "<<imuData_.ori_[3]<<std::endl;
  // std::cout<<imuData_.angularVel_[0]<<" "<<imuData_.angularVel_[1]<<" "<<imuData_.angularVel_[2]<<std::endl;
  // std::cout<<imuData_.linearAcc_[0]<<" "<<imuData_.linearAcc_[1]<<" "<<imuData_.linearAcc_[2]<<std::endl;
  // std::cout<<"===============\n";
  #ifdef TIMER
  auto read_end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(read_end_time - read_start_time);
  if (duration.count() > 500){
    std::cout<<"read time: "<<duration.count()<<" microseconds\n";
  }
  #endif
}

void GR1HW::write(const ros::Time& time, const ros::Duration& /*period*/) {
  #ifdef TIMER
  auto write_start_time = std::chrono::steady_clock::now();
  #endif
  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    write_joint_pos[i] = jointData_[i].posDes_;
    write_joint_vel[i] = jointData_[i].velDes_;
    write_joint_torq[i] = jointData_[i].ff_;
    // std::cout<<"write joint "<<i<<" pos: "<< write_joint_pos[i] << "vel: "<< write_joint_vel[i]<< "torq:" << write_joint_torq[i] <<"\n";
  }

  serial_to_parallel();

  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    // std::cout<<"write joint "<<i<<" pos: "<< write_joint_pos[i] << "torq:" << write_joint_torq[i] <<"\n";
    write_joint_pos[i] = (write_joint_pos[i] - pos_offset[i]) * 180/PI * motor_dir[i];
    write_joint_vel[i] = write_joint_vel[i] * 180/PI * motor_dir[i];
    write_joint_current[i] = torque_to_current(i, write_joint_torq[i]);

    double filtered_current = (1 - cur_lpf_ratio) * last_cmd_cur[i] + cur_lpf_ratio * write_joint_current[i];
    last_cmd_cur[i] = filtered_current;
    double filtered_pos = (1 - pos_lpf_ratio) * last_cmd_pos[i] + pos_lpf_ratio * write_joint_pos[i];
    last_cmd_pos[i] = filtered_pos;
    double filtered_vel = (1 - vel_lpf_ratio) * last_cmd_vel[i] + vel_lpf_ratio * write_joint_vel[i];
    last_cmd_vel[i] = filtered_vel;
    

    // std::cout<<"write joint "<<i<<" pos: "<< filtered_pos << "vel: "<< filtered_vel<< "current:" << filtered_current <<"\n";

    #ifndef ESTIMATION_ONLY
    if (filtered_pos - current_motor_pos[i] > 30 || filtered_pos - current_motor_pos[i] < - 30){
      disable_all_motors();
      ROS_ERROR("pos command jump!!!! =========");
      exit(1);
    }
      fsa_list[i].SetPosition(filtered_pos, filtered_vel*0.5, filtered_current);
    #endif
    // std::cout<<"write joint "<<i<<" pos: "<< write_joint_pos[i] << "current: "<< current<< "torq:" << write_joint_torq[i] <<"\n";
    // std::cout<<i<<" pos: "<< write_joint_pos[i] - current_motor_pos[i] << 
    //         "vel: "<< write_joint_vel[i] - current_motor_vel[i]<< "current:" << current - current_motor_cur[i] <<"\n";

    // last_cmd_pos[i] = write_joint_pos[i];
  }
  #ifdef TIMER
  auto write_end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(write_end_time - write_start_time);
  if (duration.count() > 500){
    std::cout<<"write time: "<<duration.count()<<" microseconds\n";
  }
  #endif
}

bool GR1HW::disable_all_motors(){

  for(int i=0; i<fsa_list.size(); i++){
    fsa_list[i].Disable();
  }

  for(int i=0; i<arm_and_head_fsa_list.size(); i++){
    arm_and_head_fsa_list[i].Disable();
  }

  return true;
}

bool GR1HW::parallel_to_serial(){
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

  return true;
}

bool GR1HW::serial_to_parallel(){
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
  return true;
}

double GR1HW::torque_to_current(int index, double torque){
  double current = torque * motor_dir[index]/(ratio[index]*scale[index]);
  // current = std::min(current, current_bound[index]);
  // current = std::max(current, -current_bound[index]);
  return current;
}

double GR1HW::current_to_torque(int index, double current){
  double torque = current * (ratio[index]*scale[index]) * motor_dir[index];
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


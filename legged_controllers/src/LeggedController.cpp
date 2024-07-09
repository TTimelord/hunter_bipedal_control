//
// Created by qiayuan on 2022/6/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/WeightedWbc.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <pluginlib/class_list_macros.hpp>
#include "std_msgs/Float64MultiArray.h"
#include "legged_controllers/utilities.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged
{
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
      leggedInterface_->getPinocchioInterface(), pinocchioMapping, leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{
    "l_hip_roll", "l_hip_yaw", "l_hip_pitch", "l_knee_pitch", "l_ankle_pitch", "l_ankle_roll",
    "r_hip_roll", "r_hip_yaw", "r_hip_pitch", "r_knee_pitch", "r_ankle_pitch", "r_ankle_roll"
  };
  for (const auto& joint_name : joint_names)
  {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("imu_link");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(),
                                       leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // wbc_ = std::make_shared<HierarchicalWbc>(leggedInterface_->getPinocchioInterface(),
  //                                     leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);

  wbc_->setStanceMode(true);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  // Configuring the hardware interface
  eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());

  // Reading relevant parameters
  RetrievingParameters();

  // loadEigenMatrix
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", defalutJointPos_);
  loadData::loadCppDataType(referenceFile, "comHeight", comHeight_);
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "stanceModeParams.";
  loadData::loadPtreeValue(pt, stance_filter_max_duration, prefix + "stanceFilterDuration", verbose);
  loadData::loadPtreeValue(pt, stance_pos_offset, prefix + "stancePosOffset", verbose);

  // Configuring an inverse kinematics processing object
  inverseKinematics_.setParam(std::make_shared<PinocchioInterface>(leggedInterface_->getPinocchioInterface()),
                              std::make_shared<CentroidalModelInfo>(leggedInterface_->getCentroidalModelInfo()));

  debug_joint_pub = controller_nh.advertise<sensor_msgs::JointState>("/debug_joint_states", 1);
  joint_state_gr1.name.resize(jointDim_);
  joint_state_gr1.effort.resize(jointDim_);
  joint_state_gr1.velocity.resize(jointDim_);
  joint_state_gr1.position.resize(jointDim_);

  // csv log file
  std::time_t now = std::time(nullptr);
  std::tm* timeInfo = std::localtime(&now);
  char timeStr[20];
  std::strftime(timeStr, sizeof(timeStr), "%Y-%m-%d_%H-%M-%S", timeInfo);
  const char* homeDir = getenv("HOME");
  if (homeDir == nullptr) {
      std::cerr << "Failed to get home directory." << std::endl;
      std::exit(EXIT_FAILURE);
  }
  filename = std::string(homeDir) + "/mpc_log/output_" + std::string(timeStr) + ".csv";
  csvFile.open(filename, std::ios::out);
  if (!csvFile.is_open()) {
    std::cerr << "Failed to open the file for writing: " << filename << std::endl;
    // throw std::runtime_error("Unable to open file for writing.");
    std::exit(EXIT_FAILURE);
  }
  return true;
}

void LeggedController::starting(const ros::Time& time)
{
  startingTime_.fromSec(time.toSec() - 0.0001);
  const ros::Time shifted_time = time - startingTime_;
  // Initial state
  currentObservation_.state.setZero(stateDim_);
  currentObservation_.input.setZero(inputDim_);
  currentObservation_.state.segment(6 + 6, jointDim_) = defalutJointPos_;
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });

  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);

  // mpcRunning_ = false;

  // Mode Subscribe
  ModeSubscribe();

  // Dynamic server
  serverPtr_ =
      std::make_unique<dynamic_reconfigure::Server<legged_controllers::TutorialsConfig>>(ros::NodeHandle("controller"));
  dynamic_reconfigure::Server<legged_controllers::TutorialsConfig>::CallbackType f;
  f = boost::bind(&LeggedController::dynamicParamCallback, this, _1, _2);
  serverPtr_->setCallback(f);

  loadControllerFlag_ = false;
  mpcRunning_ = false;
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period)
{
  const ros::Time shifted_time = time - startingTime_;
  // State Estimate
  updateStateEstimation(shifted_time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Evaluate the current policy
  vector_t optimizedState(stateDim_), optimizedInput(inputDim_);

  size_t plannedMode = 3;
  bool mpc_updated_ = false;
  if (firstStartMpc_)
  {
    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState,
                                     optimizedInput, plannedMode);
    currentObservation_.input = optimizedInput;
    mpc_updated_ = true;
  }

  // if(loadControllerFlag_){
  //   // state switch
  //   if (setWalkFlag_){
  //     if(stance_flag){
  //       if(mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time) != 3
  //         || mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time + 0.2) != 3
  //         || mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time + 0.4) != 3
  //       ){
  //         stance_flag = false;
  //       }
  //     }
  //     else{
  //       if(mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time) == 3){
  //         if(mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time + 0.4) == 3){
  //           should_start_stance = true;
  //         }
  //       }
  //     }
  //   }
  //   else{
  //     if(!stance_flag){
  //       should_start_stance = true;
  //     }
  //   }

  //   if(should_start_stance && (!stance_flag)){
  //     stance_flag = true;
  //     should_start_stance = false;

  //     //update stance start time
  //     stance_start_time = currentObservation_.time;
  //     stance_start_body_pose = currentObservation_.state.segment<6>(6);
    
  //     const auto& model = leggedInterface_->getPinocchioInterface().getModel();
  //     auto& data = leggedInterface_->getPinocchioInterface().getData();
  //     const vector_t& init_q = optimizedState.tail(gencoordDim_);
  //     pinocchio::framesForwardKinematics(model, data, init_q);
  //     for (int leg = 0; leg < 4; leg++)
  //     {
  //       auto FRAME_ID = leggedInterface_->getCentroidalModelInfo().endEffectorFrameIndices[leg];
  //       // int index = leg2index(leg);
  //       feet_pos[leg] = data.oMf[FRAME_ID].translation();
  //       feet_R[leg] = data.oMf[FRAME_ID].rotation();
  //     }

  //     // calculate stance body pose according to feet positions.
  //     stance_body_pose.segment<3>(0) = (feet_pos[0] + feet_pos[1] + feet_pos[2] + feet_pos[3]) / 4;
  //     stance_body_pose(3) = stance_start_body_pose(3);
  //     stance_body_pose(0) += stance_pos_offset*cos(stance_body_pose(3));
  //     stance_body_pose(1) += stance_pos_offset*sin(stance_body_pose(3));
  //     stance_body_pose(2) = comHeight_;
            
  //   }

  //   if(stance_flag){  //if stance then modify optimized state
  //     wbc_->setStanceMode(true);
  //     plannedMode = 3;

  //     scalar_t filter_ratio = std::min(currentObservation_.time - stance_start_time, stance_filter_max_duration)/stance_filter_max_duration;
  //     // scalar_t filter_ratio = 1.0;
      
  //     optimizedState.setZero();
  //     optimizedInput.setZero();

  //     optimizedState.segment<3>(6) = filter_ratio*stance_body_pose.segment<3>(0) + (1-filter_ratio)*stance_start_body_pose.segment<3>(0);;
  //     optimizedState(9) = stance_start_body_pose(3);
  //     optimizedState.tail<12>() = defalutJointPos_;

  //     if(filter_ratio < 1.0)
  //     for (int leg = 0; leg < 2; leg++)
  //     {
  //       int index = InverseKinematics::leg2index(leg);
  //       optimizedState.segment<6>(12 + index) = 
  //           inverseKinematics_.computeIK(optimizedState.tail<18>(), leg, feet_pos[leg], feet_R[leg]);
  //     }
  //   }
  //   else{
  //     wbc_->setStanceMode(false);
  //   }

    if (setWalkFlag_)
    {
      wbc_->setStanceMode(false);
  //     if(mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time) == 3 && 
  //         mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time + 0.2) == 3){
  //           //stance
  //       if(!stance_flag){
  //         stance_flag = true;
  //         stance_start_time = currentObservation_.time;
  //         stance_start_body_pose = currentObservation_.state.segment<6>(6);
  //         stance_body_pose.setZero();
  //         feet_array_t<vector3_t> current_feet_positions = leggedInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner()->getCurrentFeetPosition();
  //         stance_body_pose.segment<3>(0) = (current_feet_positions[0] + current_feet_positions[1] + current_feet_positions[2] + current_feet_positions[3]) / 4;
  //         stance_body_pose(3) = currentObservation_.state(9);
  //         stance_body_pose(0) -= 0.05*cos(stance_body_pose(3));
  //         stance_body_pose(1) -= 0.05*sin(stance_body_pose(3));
  //         stance_body_pose(2) = 0.88;
  //       }
  //       // geometry_msgs::Point msg;
  //       // msg.x = stance_body_pose(0);
  //       // msg.y = stance_body_pose(1);
  //       // msg.z = stance_body_pose(2);
  //       // stanceBodyPositionPublisher_.publish(msg);

  //       // scalar_t filter_ratio = std::min(currentObservation_.time - stance_start_time, stance_filter_max_duration)/stance_filter_max_duration;
  //       scalar_t filter_ratio = 1.0;
        
  //       optimizedState.setZero();
  //       optimizedInput.setZero();

  //       optimizedState.segment<3>(6) = filter_ratio*stance_body_pose.segment<3>(0) + (1-filter_ratio)*stance_start_body_pose.segment<3>(0);
  //       optimizedState(9) = stance_body_pose(3);
  //       optimizedState.segment<12>(12) = defalutJointPos_;
  //       // std::cout<<"stance_body_pose:\n"<<stance_body_pose<<"\n";
  //       // std::cout<<"currentObservation_:\n"<<currentObservation_.state.segment<3>(6)<<"\n=========\n";
  //     }
  //     else{
  //       if(stance_flag){
  //         stance_flag = false;
  //       }
  //     }
    }
    else
    {
      optimizedState.setZero();
      optimizedInput.setZero();
      optimizedState(8) = 0.88;
      optimizedState.segment(6 + 6, jointDim_) = defalutJointPos_;
      plannedMode = 3;
      wbc_->setStanceMode(true);
    }
  // }
  // else{
  //     optimizedState.setZero();
  //     optimizedInput.setZero();
  //     optimizedState(8) = 0.88;
  //     optimizedState.segment(6 + 6, jointDim_) = defalutJointPos_;
  //     plannedMode = 3;
  //     wbc_->setStanceMode(true);
  // }
  
  const vector_t& mpc_planned_body_pos = optimizedState.segment(6, 6);
  const vector_t& mpc_planned_joint_pos = optimizedState.segment(6 + 6, jointDim_);
  const vector_t& mpc_planned_joint_vel = optimizedInput.segment(12, jointDim_);

  // WBC
  wbcTimer_.startTimer();
  vector_t x;
  x.setZero(6+jointDim_+6*3+jointDim_);
  if(loadControllerFlag_){
    x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
    if(setWalkFlag_){
    csvFile << currentObservation_.time << ",";
    for (int i=0;i<currentObservation_.state.size();i++){
      csvFile << currentObservation_.state(i)<< ",";
    }
    for (int i=0; i<hybridJointHandles_.size();i++){
      csvFile << hybridJointHandles_[i].getVelocity()<< ",";;
    }
    csvFile << std::endl;
    }
  }
  const vector_t& wbc_planned_torque = x.tail(jointDim_);
  const vector_t& wbc_planned_joint_acc = x.segment(6, jointDim_);
  const vector_t& wbc_planned_body_acc = x.head(6);
  const vector_t& wbc_planned_contact_force = x.segment(6 + jointDim_, wbc_->getContactForceSize());
  wbcTimer_.endTimer();
  const scalar_t wbc_time = wbcTimer_.getLastIntervalInMilliseconds();
  // if (wbc_time > 1){
  //   std::cout<<wbc_time<<std::endl;
  // }

  posDes_ = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  velDes_ = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  scalar_t dt = period.toSec();
  // posDes_ = posDes_ + 0.5 * wbc_planned_joint_acc * dt * dt;
  // velDes_ = velDes_ + wbc_planned_joint_acc * dt;


  // joint_state_gr1.header.stamp = ros::Time::now();
  // for(int i=0; i<jointDim_;i++){
  //   joint_state_gr1.effort[i] = wbc_planned_torque(i);
  // }
  // // joint_state_gr1.position[0] = currentObservation_.state(7);
  // // joint_state_gr1.position[1] = currentObservation_.state(8);
  // debug_joint_pub.publish(joint_state_gr1);

  // std::cout<<"WBC start =========="<<std::endl;
  // std::cout<<"stance body pose"<<stance_body_pose<<std::endl;
  // std::cout<<posDes_<<std::endl;
  // std::cout<<"observation"<<std::endl;
  // std::cout<<currentObservation_.state.segment<6>(6)<<std::endl;
  // std::cout<<"optimized state\n";
  // std::cout<<optimizedState.segment<6>(6)<<std::endl;
  // std::cout<<"wbc_planned_torque"<<std::endl;
  // std::cout<<wbc_planned_torque<<std::endl;
  // std::cout<<"wbc_planned_joint_acc"<<std::endl;
  // std::cout<<wbc_planned_joint_acc<<std::endl;
  // std::cout<<"wbc_planned_body_acc"<<std::endl;
  // std::cerr<<wbc_planned_body_acc<<std::endl;
  // std::cout<<"wbc_planned_contact_force"<<std::endl;
  // std::cout<<wbc_planned_contact_force<<std::endl;
  // std::cout<<"WBC end =========="<<std::endl;

  vector_t output_torque(jointDim_);
  //*********************** Set Joint Command: Normal Tracking *****************************//
  for (size_t j = 0; j < jointDim_; ++j)
  {
    //"Limit protection
    const auto& model = leggedInterface_->getPinocchioInterface().getModel();
    double lower_bound = model.lowerPositionLimit(6 + j);
    double upper_bound = model.upperPositionLimit(6 + j);
    if (!emergencyStopFlag_ && loadControllerFlag_ &&
        (hybridJointHandles_[j].getPosition() > upper_bound + 0.02 ||
         hybridJointHandles_[j].getPosition() < lower_bound - 0.02))
    {
      emergencyStopFlag_ = true;
      std::cerr << "Reach Position Limit!!!!!!!!!!!!!!!!!!!!!!!! " << j << ":" << hybridJointHandles_[j].getPosition()
                << std::endl;
    }
    if (!loadControllerFlag_)
    {
      if(j==0 || j==1 || j==2 || j==6 || j==7 || j==8)
      {
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_position,
                                          0);
      }
      else if(j==3 || j==9) // knee
      {
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_position, 0);
      }
      else if (j == 4 || j == 10) //ankle_pitch
      {
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_feet, 0);
      }
      else
      {
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_feet, 0);
      }
    }
    else
    {
      contact_flag_t cmdContactFlag = modeNumber2StanceLeg(
          mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time));
      if (j == 0 || j == 1 || j == 6 || j == 7)
      {
        hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],
                                          cmdContactFlag[int(j / 6)] ? kp_small_stance : kp_small_swing, kd_small,
                                          wbc_planned_torque(j));
      }
      else if (j== 4 || j == 10)
      {
        hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],
                                          cmdContactFlag[int(j / 6)] ? kp_small_stance : kp_small_swing, kd_feet,
                                          wbc_planned_torque(j));
      }
      else if (j== 5 || j == 11)
      {
        hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],
                                          cmdContactFlag[int(j / 6)] ? kp_small_stance : kp_small_swing, kd_feet,
                                          wbc_planned_torque(j));
      }
      else
      {    // hip and knee pitch
        hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],
                                          cmdContactFlag[int(j / 6)] ? kp_big_stance : kp_big_swing, kd_big,
                                          wbc_planned_torque(j));
      }
      // {
      //   hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],
      //                                     0, 0,
      //                                     wbc_planned_torque(j));
      // }
      // ROS_INFO("wbc torque %i: %f", j, wbc_planned_torque(j)); 
      // ROS_INFO("pos %i: %f", j, posDes_[j]);
    }
    if (emergencyStopFlag_)
    {
      hybridJointHandles_[j].setCommand(0, 0, 0, 1, 0);
    }
    posDesOutput_(j) = hybridJointHandles_[j].getPositionDesired();
    velDesOutput_(j) = hybridJointHandles_[j].getVelocityDesired();

    output_torque(j) = hybridJointHandles_[j].getFeedforward() +
                       hybridJointHandles_[j].getKp() *
                           (hybridJointHandles_[j].getPositionDesired() - hybridJointHandles_[j].getPosition()) +
                       hybridJointHandles_[j].getKd() *
                           (hybridJointHandles_[j].getVelocityDesired() - hybridJointHandles_[j].getVelocity());
  }
  //*********************** Set Joint Command: Torque Tracking Test *****************************//

  CommandData command_data;
  vector_t planned_state = currentObservation_.state;
  vector_t planned_input = currentObservation_.input;
  planned_state.tail(jointDim_) = posDesOutput_;
  planned_input.tail(jointDim_) = velDesOutput_;
  command_data.mpcTargetTrajectories_.timeTrajectory.push_back(currentObservation_.time);
  command_data.mpcTargetTrajectories_.stateTrajectory.push_back(planned_state);
  command_data.mpcTargetTrajectories_.inputTrajectory.push_back(planned_input);
  command_data = mpc_updated_ ? mpcMrtInterface_->getCommand() : command_data;
  PrimalSolution primal_solution = mpc_updated_ ? mpcMrtInterface_->getPolicy() : PrimalSolution();

  // Visualization
  robotVisualizer_->update(currentObservation_, primal_solution, command_data,
                           leggedInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner());
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period)
{
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size()),
      jointTor(hybridJointHandles_.size());
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t cmdContactFlag, cmdContactFlagBefore, delayedContactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
  {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
    jointTor(i) = hybridJointHandles_[i].getEffort();
  }

  cmdContactFlag = modeNumber2StanceLeg(
      mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time));

  cmdContactFlagBefore = modeNumber2StanceLeg(
    mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time - 0.07));

  for(size_t i = 0; i < cmdContactFlag.size(); ++i){
    if(cmdContactFlag[i] && cmdContactFlagBefore[i]){
      delayedContactFlag[i] = true;
    }
    else{
      delayedContactFlag[i] = false;
    }
  }

  if (!firstStartMpc_)
  {
    for (size_t i = 0; i < cmdContactFlag.size(); ++i)
    {
      cmdContactFlag[i] = true;
    }
  }
  stateEstimate_->updateCmdContact(cmdContactFlag);
  stateEstimate_->setStartStopTime4Legs(
      leggedInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner()->threadSaftyGetStartStopTime(
          currentObservation_.time));

  for (size_t i = 0; i < 4; ++i)
  {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }

  for (size_t i = 0; i < 3; ++i)
  {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }

  for (size_t i = 0; i < 9; ++i)
  {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(delayedContactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance,
                            linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);

  currentObservation_.time = time.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state.head(stateDim_) = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state.segment<3>(3) = linear_velocity_lpf_ratio*currentObservation_.state.segment<3>(3) + (1-linear_velocity_lpf_ratio)*last_linear_velocity;
  last_linear_velocity = currentObservation_.state.segment<3>(3);

  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();

  const auto& reference_manager = leggedInterface_->getSwitchedModelReferenceManagerPtr();
  reference_manager->getSwingTrajectoryPlanner()->setBodyVelWorld(stateEstimate_->getBodyVelWorld());
  reference_manager->setEstContactFlag(cmdContactFlag);

  stateEstimate_->setCmdTorque(jointTor);
  stateEstimate_->estContactForce(period);

  auto remove_gravity = linearAccel;
  remove_gravity(2) -= 9.81;
}

LeggedController::~LeggedController()
{
  controllerRunning_ = false;
  mpcRunning_ = false;
  if (mpcThread_.joinable())
  {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";

  if (csvFile.is_open()) {
      csvFile.close();
      ROS_INFO("log file saved");
  }
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                            const std::string& referenceFile, bool verbose)
{
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc()
{
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  auto rosReferenceManagerPtr =
      std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
  stanceBodyPositionPublisher_ = nh.advertise<geometry_msgs::Point>("stance_body_position", 1);
}

void LeggedController::setupMrt()
{
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();
  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_)
    {
      try
      {
        executeAndSleep(
            [&]() {
              if (mpcRunning_)
              {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
                firstStartMpc_ = true;
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      }
      catch (const std::exception& e)
      {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose)
{
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  stateEstimate_->loadSettings(taskFile, verbose);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
  last_linear_velocity.setZero();
  linear_velocity_lpf_ratio = 0.3;
}

void LeggedController::dynamicParamCallback(legged_controllers::TutorialsConfig& config, uint32_t level)
{
  kp_position = config.kp_position;
  kd_position = config.kd_position;

  kp_big_stance = config.kp_big_stance;
  kp_big_swing = config.kp_big_swing;

  kp_small_stance = config.kp_small_stance;
  kp_small_swing = config.kp_small_swing;
  kd_small = config.kd_small;
  kd_big = config.kd_big;

  kd_feet = config.kd_feet;
}

void LeggedController::RetrievingParameters()
{
  stateDim_ = leggedInterface_->getCentroidalModelInfo().stateDim;
  inputDim_ = leggedInterface_->getCentroidalModelInfo().inputDim;
  jointDim_ = leggedInterface_->getCentroidalModelInfo().actuatedDofNum;
  footDim_ = leggedInterface_->getCentroidalModelInfo().numThreeDofContacts;
  gencoordDim_ = leggedInterface_->getCentroidalModelInfo().generalizedCoordinatesNum;
  dofPerLeg_ = jointDim_ / 2;
  defalutJointPos_.resize(jointDim_);
}

void LeggedController::resetMPC()
{
  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });
  mpcMrtInterface_->resetMpcNode(target_trajectories);
}
void LeggedController::ModeSubscribe()
{
  subSetWalk_ =
      ros::NodeHandle().subscribe<std_msgs::Float32>("/set_walk", 1, &LeggedController::setWalkCallback, this);
  subLoadcontroller_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/load_controller", 1,
                                                                      &LeggedController::loadControllerCallback, this);
  subEmgstop_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/emergency_stop", 1,
                                                               &LeggedController::EmergencyStopCallback, this);
  subResetTarget_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/reset_estimation", 1,
                                                                  &LeggedController::ResetTargetCallback, this);
}

void LeggedController::EmergencyStopCallback(const std_msgs::Float32::ConstPtr& msg)
{
  emergencyStopFlag_ = true;
  ROS_INFO("Successfully load the controller");
}

void LeggedController::setWalkCallback(const std_msgs::Float32::ConstPtr& msg)
{
  setWalkFlag_ = true;
  ROS_INFO("Set WALK Mode");
}

void LeggedController::loadControllerCallback(const std_msgs::Float32::ConstPtr& msg)
{
  loadControllerFlag_ = true;
  mpcRunning_ = true;
  ROS_INFO("Successfully load the controller");
}
void LeggedController::ResetTargetCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Initial state
  currentObservation_.state.setZero(stateDim_);
  currentObservation_.input.setZero(inputDim_);
  currentObservation_.state.segment(6 + 6, jointDim_) = defalutJointPos_;
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });

  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO("Reset the target");

} 
}// namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
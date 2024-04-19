//
// Created by qiayuan on 22-12-23.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "legged_wbc/WeightedWbc.h"

#include <qpOASES.hpp>

namespace legged
{
vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired,
                             const vector_t& rbdStateMeasured, size_t mode, scalar_t period)
{
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  // Constraints
  Task constraints = formulateConstraints();
  size_t numConstraints = constraints.b_.size() + constraints.f_.size();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
  vector_t lbA(numConstraints), ubA(numConstraints);  // clang-format off
  A << constraints.a_,
       constraints.d_;

  lbA << constraints.b_,
         -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
  ubA << constraints.b_,
         constraints.f_;  // clang-format on

  // Cost
  Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H =
      weighedTask.a_.transpose() * weighedTask.a_;
  vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

  // Solve
  auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  qpProblem.setOptions(options);
  int nWsr = 20;

  qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
  vector_t qpSol(getNumDecisionVars());

  qpProblem.getPrimalSolution(qpSol.data());

  if (!qpProblem.isSolved())
  {
    std::cout << "ERROR: WeightWBC Not Solved!!!" << std::endl;
    if (last_qpSol.size() > 0)
      qpSol = last_qpSol;
  }

  last_qpSol = qpSol;
  return qpSol;
}

Task WeightedWbc::formulateConstraints()
{
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask();
}

Task WeightedWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period)
{
  if (stance_mode_)
    return formulateStanceBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_;
  else
    return formulateSwingLegTask() * weightSwingLeg_ +
           formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
           formulateContactForceTask(inputDesired) * weightContactForce_;
}

Task WeightedWbc::formulateStanceBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired,
                                               scalar_t period)
{
  return formulateBaseAccelTask(stateDesired, inputDesired, period);
//   matrix_t a(6, numDecisionVars_);
//   a.setZero();
//   a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);
//   // a.block(9, 9, 10, 10) = matrix_t::Identity(1, 1);
//   // a.block(15, 15, 16, 16) = matrix_t::Identity(1, 1);

//   vector6_t b;
//   vector6_t stance_q_desired;
//   b.setZero();

//   stance_q_desired.setZero();
//   stance_q_desired(2) = 0.88;
//   stance_q_desired(0) = -0.05;

//   vector3_t eulerAngles = qMeasured_.segment<3>(3);

//   // from derivative euler to angular
//   vector3_t vMeasuredGlobal =
//       getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vMeasured_.segment<3>(3));

//   // from euler to rotation
//   vector3_t eulerAnglesDesired = stance_q_desired.tail<3>();
//   matrix3_t rotationBaseMeasuredToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAngles);
//   matrix3_t rotationBaseReferenceToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAnglesDesired);

//   vector3_t error = rotationErrorInWorld<scalar_t>(rotationBaseReferenceToWorld, rotationBaseMeasuredToWorld);

//   b.block(0, 0, 2, 1) = com_kp_ * (stance_q_desired.segment<2>(0) - qMeasured_.segment<2>(0)) +
//         com_kd_ * ( - vMeasured_.segment<2>(0));
//   b(2) = baseHeightKp_*(stance_q_desired(2) - qMeasured_(2)) + baseHeightKd_*(0 - vMeasured_(2));
//   b.block(3, 0, 3, 1) = baseAngularKp_ * error + baseAngularKd_ * ( - vMeasuredGlobal);

//   return { a, b, matrix_t(), vector_t() };
}

void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose)
{
  WbcBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";
  if (verbose)
  {
    std::cerr << "\n #### WBC weight:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
}

}  // namespace legged

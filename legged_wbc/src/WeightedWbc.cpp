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
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask(); //formulateNoContactMotionTask();
}

Task WeightedWbc::formulateZeroContactForceTask()
{
  int num_constrained_contact = info_.numThreeDofContacts - 4;
  int num_constrained_forces = 3 * num_constrained_contact;
  matrix_t a(num_constrained_forces, numDecisionVars_);
  a.setZero();
  for(int i = 0; i < num_constrained_contact; i++){
    a.block(0 + 3*i, 6 + info_.actuatedDofNum + 4*3 + 3*i, 2, 2).setIdentity();  // only constrain Z axis
  }
  // std::cout<<a.block(0, 6 + info_.actuatedDofNum + 4*3, num_constrained_forces, num_constrained_forces)<<std::endl;
  // exit(0);
  // a.block(0, 6 + info_.actuatedDofNum, num_constrained_forces, num_constrained_forces).setIdentity();

  vector_t b(num_constrained_forces);
  b.setZero();    

  return { a, b, matrix_t(), vector_t() };
}


Task WeightedWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period)
{
  if (stance_mode_)
    return formulateStanceBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
            formulateNoContactMotionTask() * weightNoContactMotion_;
  else
    return formulateSwingLegTask() * weightSwingLeg_ +
           formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
           formulateContactForceTask(inputDesired) * weightContactForce_ +
           formulateNoContactMotionTask() * weightNoContactMotion_;
}

// Task WeightedWbc::formulateStanceBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period)
// {
//   matrix_t a(6, numDecisionVars_);
//   a.setZero();
//   a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

//   vector6_t b;
//   b.setZero();
//   return { a, b, matrix_t(), vector_t() };
// }

Task WeightedWbc::formulateMinimizeTorqueTask()
{
  matrix_t a(12, numDecisionVars_);
  a.setZero();
  a.block(0, 6+12+18, 12, 12).setIdentity();
  a(4, 6+12+18+4) = 3;
  a(10, 6+12+18+10) = 3;

  Eigen::Matrix<scalar_t, 12, 1> b;
  b.setZero();
  return { a, b, matrix_t(), vector_t() };
}

Task WeightedWbc::formulateContactForceRegularizationTask()
{

  matrix_t a(info_.numThreeDofContacts * 3, numDecisionVars_);
  a.setZero();
  a.block(0, 6 + info_.actuatedDofNum, info_.numThreeDofContacts * 3, info_.numThreeDofContacts * 3)
           = matrix_t::Identity(info_.numThreeDofContacts * 3, info_.numThreeDofContacts * 3);

  vector_t b(info_.numThreeDofContacts * 3);
  b.setZero();

  return { a, b, matrix_t(), vector_t() };
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
  // loadData::loadPtreeValue(pt, weightContactForceRegularization_, prefix + "contactForceRegularization", verbose);
  loadData::loadPtreeValue(pt, weightNoContactMotion_, prefix + "noContactMotion", verbose);
}

}  // namespace legged

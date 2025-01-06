
#pragma once

#include "motion_control/legged_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include "motion_control/gait/MotionPhaseDefinition.h"//#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>


// Decision Variables: x = [\dot u^T, F^T, \tau^T]^T
class WbcBase {
  using Vector6 = Eigen::Matrix<ocs2::scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<ocs2::scalar_t, 6, 6>;

 public:
  WbcBase(const ocs2::PinocchioInterface& pinocchioInterface, ocs2::CentroidalModelInfo info, const ocs2::PinocchioEndEffectorKinematics& eeKinematics);

  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  virtual ocs2::vector_t update(const ocs2::vector_t& stateDesired, const ocs2::vector_t& inputDesired, const ocs2::vector_t& rbdStateMeasured, size_t mode,
                          ocs2::scalar_t period);

 protected:
  void updateMeasured(const ocs2::vector_t& rbdStateMeasured);
  void updateDesired(const ocs2::vector_t& stateDesired, const ocs2::vector_t& inputDesired);

  size_t getNumDecisionVars() const { return numDecisionVars_; }

  Task formulateFloatingBaseEomTask();
  Task formulateTorqueLimitsTask();
  Task formulateNoContactMotionTask();
  Task formulateFrictionConeTask();
  Task formulateBaseAccelTask(const ocs2::vector_t& stateDesired, const ocs2::vector_t& inputDesired, ocs2::scalar_t period);
  Task formulateSwingLegTask();
  Task formulateContactForceTask(const ocs2::vector_t& inputDesired) const;

  size_t numDecisionVars_;
  ocs2::PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  ocs2::CentroidalModelInfo info_;

  std::unique_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematics_;
  ocs2::CentroidalModelPinocchioMapping mapping_;

  ocs2::vector_t qMeasured_, vMeasured_, inputLast_;
  ocs2::matrix_t j_, dj_;
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // Task Parameters:
  ocs2::vector_t torqueLimits_;
  ocs2::scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
};


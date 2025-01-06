/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

// ocs2
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_sqp/SqpSettings.h>

#include "motion_control/common/ModelSettings.h"// #include "ocs2_legged_robot/common/ModelSettings.h"
#include "motion_control/initialization/LeggedRobotInitializer.h"//#include "ocs2_legged_robot/initialization/LeggedRobotInitializer.h"
#include "motion_control/reference_manager/SwitchedModelReferenceManager.h" //#include "ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h"


/**
 * LeggedRobotInterface class
 * General interface for mpc implementation on the legged robot model
 */

class LeggedRobotInterface final : public ocs2::RobotInterface {
 public:
  /**
   * Constructor
   *
   * @throw Invalid argument error if input task file or urdf file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] urdfFile: The absolute path to the URDF file for the robot.
   * @param [in] referenceFile: The absolute path to the reference configuration file.
   * @param [in] useHardFrictionConeConstraint: Which to use hard or soft friction cone constraints.
   */
  LeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                       bool useHardFrictionConeConstraint = false);

  ~LeggedRobotInterface() override = default;

  const ocs2::OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

  const ModelSettings& modelSettings() const { return modelSettings_; }
  const ocs2::ddp::Settings& ddpSettings() const { return ddpSettings_; }
  const ocs2::mpc::Settings& mpcSettings() const { return mpcSettings_; }
  const ocs2::rollout::Settings& rolloutSettings() const { return rolloutSettings_; }
  const ocs2::sqp::Settings& sqpSettings() { return sqpSettings_; }
  const ocs2::ipm::Settings& ipmSettings() { return ipmSettings_; }

  const ocs2::vector_t& getInitialState() const { return initialState_; }
  const ocs2::RolloutBase& getRollout() const { return *rolloutPtr_; }
  ocs2::PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
  const ocs2::CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }
  std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

  const LeggedRobotInitializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

 private:
  // set up the Optimal Conrol Problem based on the taskFile,urdfFile,referenceFile
  void setupOptimalConrolProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);
  //load initial GaitSchedule from referenceFile
  std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& file, bool verbose) const; 
  //load Q and R from taskfile
  std::unique_ptr<ocs2::StateInputCost> getBaseTrackingCost(const std::string& taskFile, const ocs2::CentroidalModelInfo& info, bool verbose);
  //compute initial R based on taskfile and baseToFeetJacobians
  ocs2::matrix_t initializeInputCostWeight(const std::string& taskFile, const ocs2::CentroidalModelInfo& info);
  //load Friction parameters from taskFile
  std::pair<ocs2::scalar_t, ocs2::RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string& taskFile, bool verbose) const;
  //get FrictionCone Constraint based on Friction parameters
  std::unique_ptr<ocs2::StateInputConstraint> getFrictionConeConstraint(size_t contactPointIndex, ocs2::scalar_t frictionCoefficient);
  //get soft FrictionCone Constraint based on Friction parameters
  std::unique_ptr<ocs2::StateInputCost> getFrictionConeSoftConstraint(size_t contactPointIndex, ocs2::scalar_t frictionCoefficient,
                                                                const ocs2::RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
  //get Zero Force Constraint for each legs, the Constraint will be computed based of the contact flag of each leg
  std::unique_ptr<ocs2::StateInputConstraint> getZeroForceConstraint(size_t contactPointIndex);
  //get Zero Velocity Constraint for each legs based on End Effector Kinematics
  std::unique_ptr<ocs2::StateInputConstraint> getZeroVelocityConstraint(const ocs2::EndEffectorKinematics<ocs2::scalar_t>& eeKinematics,
                                                                  size_t contactPointIndex, bool useAnalyticalGradients);
  //get Zero Normal Velocity Constraint for each legs based on End Effector Kinematics
  std::unique_ptr<ocs2::StateInputConstraint> getNormalVelocityConstraint(const ocs2::EndEffectorKinematics<ocs2::scalar_t>& eeKinematics,
                                                                    size_t contactPointIndex, bool useAnalyticalGradients);

  ModelSettings modelSettings_; // preserve some model data, including jointNames, contactNames3DoF, positionErrorGain(?), phaseTransitionStanceTime(?) and so on
  ocs2::ddp::Settings ddpSettings_; // settings from task.info
  ocs2::mpc::Settings mpcSettings_; // settings from task.info
  ocs2::sqp::Settings sqpSettings_; // settings from task.info
  ocs2::ipm::Settings ipmSettings_; // settings from task.info
  const bool useHardFrictionConeConstraint_; //if use hard friction cone constraint on contact forces.

  std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_; //ocs2 pinocchio Interface
  ocs2::CentroidalModelInfo centroidalModelInfo_; // robot centroidal model key information
  // centroidal model describes the motion and dynamic behavior of an object centered around its centroid (center of mass).

  std::unique_ptr<ocs2::OptimalControlProblem> problemPtr_; // ocs2 OptimalControl interface
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_; // preserve the preserve the gait schedule and the height sequence of each leg during the gait time period.

  ocs2::rollout::Settings rolloutSettings_; // Rollout settings from task.info
  std::unique_ptr<ocs2::RolloutBase> rolloutPtr_; //forward integrating the system dynamics over a specified time interval using the given control policy and initial state. It outputs the final state, time trajectory, state trajectory, and input trajectory.
  std::unique_ptr<LeggedRobotInitializer> initializerPtr_; // compute the input force and state of each leg at the initial state

  ocs2::vector_t initialState_;
};


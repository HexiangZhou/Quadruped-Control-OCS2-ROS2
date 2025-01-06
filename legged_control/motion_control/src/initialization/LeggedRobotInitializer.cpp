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
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

#include "motion_control/initialization/LeggedRobotInitializer.h"

#include "motion_control/common/utils.h"


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInitializer::LeggedRobotInitializer(ocs2::CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager,
                                               bool extendNormalizedMomentum)
    : info_(std::move(info)), referenceManagerPtr_(&referenceManager), extendNormalizedMomentum_(extendNormalizedMomentum) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInitializer* LeggedRobotInitializer::clone() const {
  return new LeggedRobotInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotInitializer::compute(ocs2::scalar_t time, const ocs2::vector_t& state, ocs2::scalar_t nextTime, ocs2::vector_t& input, ocs2::vector_t& nextState) {
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  input = weightCompensatingInput(info_, contactFlags);
  nextState = state;
  if (!extendNormalizedMomentum_) { // extendNormalizedMomentum_==true means take momentum into consideration, otherwise clear the momentum in the state
    ocs2::centroidal_model::getNormalizedMomentum(nextState, info_).setZero();
  }
}
// compute(): compute control inputs to the legged robot to ensure that the robot remains balanced with zero joint velocity.
// First gets the number of support legs in the current contact state.
// Second Calculate the total weight of the robot due to gravity and gravity required to be applied to each leg.
// Third updates the contact force for each leg and state of robot


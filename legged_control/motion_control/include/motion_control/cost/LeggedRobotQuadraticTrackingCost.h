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

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

#include "motion_control/common/utils.h"
#include "motion_control/reference_manager/SwitchedModelReferenceManager.h"


/**
 * State-input tracking cost used for intermediate times
 */
class LeggedRobotStateInputQuadraticCost final : public ocs2::QuadraticStateInputCost {
 public:
  LeggedRobotStateInputQuadraticCost(ocs2::matrix_t Q, ocs2::matrix_t R, ocs2::CentroidalModelInfo info,
                                     const SwitchedModelReferenceManager& referenceManager)
      : QuadraticStateInputCost(std::move(Q), std::move(R)), info_(std::move(info)), referenceManagerPtr_(&referenceManager) {}

  ~LeggedRobotStateInputQuadraticCost() override = default;
  LeggedRobotStateInputQuadraticCost* clone() const override { return new LeggedRobotStateInputQuadraticCost(*this); }

 private:
  LeggedRobotStateInputQuadraticCost(const LeggedRobotStateInputQuadraticCost& rhs) = default;

  std::pair<ocs2::vector_t, ocs2::vector_t> getStateInputDeviation(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
                                                       const ocs2::TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const ocs2::vector_t xNominal = targetTrajectories.getDesiredState(time);
    const ocs2::vector_t uNominal = weightCompensatingInput(info_, contactFlags);
    return {state - xNominal, input - uNominal};
  }

  const ocs2::CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
};

/**
 * State tracking cost used for the final time
 */
class LeggedRobotStateQuadraticCost final : public ocs2::QuadraticStateCost {
 public:
  LeggedRobotStateQuadraticCost(ocs2::matrix_t Q, ocs2::CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager)
      : QuadraticStateCost(std::move(Q)), info_(std::move(info)), referenceManagerPtr_(&referenceManager) {}

  ~LeggedRobotStateQuadraticCost() override = default;
  LeggedRobotStateQuadraticCost* clone() const override { return new LeggedRobotStateQuadraticCost(*this); }

 private:
  LeggedRobotStateQuadraticCost(const LeggedRobotStateQuadraticCost& rhs) = default;

  ocs2::vector_t getStateDeviation(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const ocs2::vector_t xNominal = targetTrajectories.getDesiredState(time);
    return state - xNominal;
  }

  const ocs2::CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
};


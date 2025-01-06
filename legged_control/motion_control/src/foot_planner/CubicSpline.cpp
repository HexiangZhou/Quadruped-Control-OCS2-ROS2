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

#include "motion_control/foot_planner/CubicSpline.h"


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CubicSpline::CubicSpline(Node start, Node end) {
  assert(start.time < end.time);
  t0_ = start.time;
  t1_ = end.time;
  dt_ = end.time - start.time;

  ocs2::scalar_t dp = end.position - start.position;
  ocs2::scalar_t dv = end.velocity - start.velocity;

  dc0_ = 0.0;
  dc1_ = start.velocity;
  dc2_ = -(3.0 * start.velocity + dv);
  dc3_ = (2.0 * start.velocity + dv);

  c0_ = dc0_ * dt_ + start.position;
  c1_ = dc1_ * dt_;
  c2_ = dc2_ * dt_ + 3.0 * dp;
  c3_ = dc3_ * dt_ - 2.0 * dp;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::scalar_t CubicSpline::position(ocs2::scalar_t time) const {
  ocs2::scalar_t tn = normalizedTime(time);
  return c3_ * tn * tn * tn + c2_ * tn * tn + c1_ * tn + c0_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::scalar_t CubicSpline::velocity(ocs2::scalar_t time) const {
  ocs2::scalar_t tn = normalizedTime(time);
  return (3.0 * c3_ * tn * tn + 2.0 * c2_ * tn + c1_) / dt_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::scalar_t CubicSpline::acceleration(ocs2::scalar_t time) const {
  ocs2::scalar_t tn = normalizedTime(time);
  return (6.0 * c3_ * tn + 2.0 * c2_) / (dt_ * dt_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::scalar_t CubicSpline::startTimeDerivative(ocs2::scalar_t t) const {
  ocs2::scalar_t tn = normalizedTime(t);
  ocs2::scalar_t dCoff = -(dc3_ * tn * tn * tn + dc2_ * tn * tn + dc1_ * tn + dc0_);
  ocs2::scalar_t dTn = -(t1_ - t) / (dt_ * dt_);
  return velocity(t) * dt_ * dTn + dCoff;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::scalar_t CubicSpline::finalTimeDerivative(ocs2::scalar_t t) const {
  ocs2::scalar_t tn = normalizedTime(t);
  ocs2::scalar_t dCoff = (dc3_ * tn * tn * tn + dc2_ * tn * tn + dc1_ * tn + dc0_);
  ocs2::scalar_t dTn = -(t - t0_) / (dt_ * dt_);
  return velocity(t) * dt_ * dTn + dCoff;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::scalar_t CubicSpline::normalizedTime(ocs2::scalar_t t) const {
  return (t - t0_) / dt_;
}

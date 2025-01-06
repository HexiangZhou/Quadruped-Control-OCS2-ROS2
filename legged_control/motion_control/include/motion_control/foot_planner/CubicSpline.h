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

#include <ocs2_core/Types.h>


class CubicSpline {
 public:
  struct Node {
    ocs2::scalar_t time;
    ocs2::scalar_t position;
    ocs2::scalar_t velocity;
  };

  CubicSpline(Node start, Node end);

  ocs2::scalar_t position(ocs2::scalar_t time) const;

  ocs2::scalar_t velocity(ocs2::scalar_t time) const;

  ocs2::scalar_t acceleration(ocs2::scalar_t time) const;

  ocs2::scalar_t startTimeDerivative(ocs2::scalar_t t) const;

  ocs2::scalar_t finalTimeDerivative(ocs2::scalar_t t) const;

 private:
  ocs2::scalar_t normalizedTime(ocs2::scalar_t t) const;

  ocs2::scalar_t t0_;
  ocs2::scalar_t t1_;
  ocs2::scalar_t dt_;

  ocs2::scalar_t c0_;
  ocs2::scalar_t c1_;
  ocs2::scalar_t c2_;
  ocs2::scalar_t c3_;

  ocs2::scalar_t dc0_;  // derivative w.r.t. dt_
  ocs2::scalar_t dc1_;  // derivative w.r.t. dt_
  ocs2::scalar_t dc2_;  // derivative w.r.t. dt_
  ocs2::scalar_t dc3_;  // derivative w.r.t. dt_
};


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

#include "motion_control/gait/ModeSequenceTemplate.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadData.h>


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& stream, const ModeSequenceTemplate& modeSequenceTemplate) {
  stream << "Template switching times: {" << ocs2::toDelimitedString(modeSequenceTemplate.switchingTimes) << "}\n";
  stream << "Template mode sequence:   {" << ocs2::toDelimitedString(modeSequenceTemplate.modeSequence) << "}\n";
  return stream;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSequenceTemplate loadModeSequenceTemplate(const std::string& filename, const std::string& topicName, bool verbose) {
  // if verbose = true, then ocs2::loadData::loadStdVector will print the loaded gait information
  std::vector<ocs2::scalar_t> switchingTimes;
  ocs2::loadData::loadStdVector(filename, topicName + ".switchingTimes", switchingTimes, verbose);

  std::vector<std::string> modeSequenceString;
  ocs2::loadData::loadStdVector(filename, topicName + ".modeSequence", modeSequenceString, verbose);

  if (switchingTimes.empty() || modeSequenceString.empty()) {
    throw std::runtime_error("[loadModeSequenceTemplate] failed to load : " + topicName + " from " + filename);
  }

  // convert the mode name to mode enum
  std::vector<size_t> modeSequence;
  modeSequence.reserve(modeSequenceString.size());
  for (const auto& modeName : modeSequenceString) {
    modeSequence.push_back(string2ModeNumber(modeName));
  }

  return {switchingTimes, modeSequence};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Gait toGait(const ModeSequenceTemplate& modeSequenceTemplate) {
  const auto startTime = modeSequenceTemplate.switchingTimes.front();
  const auto endTime = modeSequenceTemplate.switchingTimes.back();
  Gait gait;
  gait.duration = endTime - startTime;
  // Events: from time -> phase
  gait.eventPhases.reserve(modeSequenceTemplate.switchingTimes.size());
  std::for_each(modeSequenceTemplate.switchingTimes.begin() + 1, modeSequenceTemplate.switchingTimes.end() - 1,
                [&](ocs2::scalar_t eventTime) { gait.eventPhases.push_back((eventTime - startTime) / gait.duration); });
  // Modes:
  gait.modeSequence = modeSequenceTemplate.modeSequence;
  assert(isValidGait(gait));
  return gait;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::ModeSchedule loadModeSchedule(const std::string& filename, const std::string& topicName, bool verbose) {
  std::vector<ocs2::scalar_t> eventTimes;
  ocs2::loadData::loadStdVector(filename, topicName + ".eventTimes", eventTimes, verbose);

  std::vector<std::string> modeSequenceString;
  ocs2::loadData::loadStdVector(filename, topicName + ".modeSequence", modeSequenceString, verbose);

  if (modeSequenceString.empty()) {
    throw std::runtime_error("[loadModeSchedule] failed to load : " + topicName + " from " + filename);
  }

  // convert the mode name to mode enum
  std::vector<size_t> modeSequence;
  modeSequence.reserve(modeSequenceString.size());
  for (const auto& modeName : modeSequenceString) {
    modeSequence.push_back(string2ModeNumber(modeName));
  }

  return {eventTimes, modeSequence};
}


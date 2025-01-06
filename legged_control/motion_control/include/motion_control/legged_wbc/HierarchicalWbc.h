#pragma once

#include "motion_control/legged_wbc/WbcBase.h"


class HierarchicalWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;

  ocs2::vector_t update(const ocs2::vector_t& stateDesired, const ocs2::vector_t& inputDesired, const ocs2::vector_t& rbdStateMeasured, size_t mode,
                  ocs2::scalar_t period) override;
};


#include "motion_control/legged_wbc/HierarchicalWbc.h"

#include "motion_control/legged_wbc/HoQp.h"

ocs2::vector_t HierarchicalWbc::update(const ocs2::vector_t& stateDesired, const ocs2::vector_t& inputDesired, const ocs2::vector_t& rbdStateMeasured, size_t mode,
                                 ocs2::scalar_t period) {
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
  Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period) + formulateSwingLegTask();
  Task task2 = formulateContactForceTask(inputDesired);
  HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

  return hoQp.getSolutions();
}


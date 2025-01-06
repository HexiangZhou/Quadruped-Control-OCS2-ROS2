#include "motion_control/legged_wbc/WbcBase.h"


class WeightedWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;

  ocs2::vector_t update(const ocs2::vector_t& stateDesired, const ocs2::vector_t& inputDesired, const ocs2::vector_t& rbdStateMeasured, size_t mode,
                  ocs2::scalar_t period) override;

  void loadTasksSetting(const std::string& taskFile, bool verbose) override;

 protected:
  virtual Task formulateConstraints();
  virtual Task formulateWeightedTasks(const ocs2::vector_t& stateDesired, const ocs2::vector_t& inputDesired, ocs2::scalar_t period);

 private:
  ocs2::scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
};

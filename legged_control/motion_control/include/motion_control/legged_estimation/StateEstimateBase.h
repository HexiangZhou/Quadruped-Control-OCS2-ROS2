#pragma once

// #include <legged_common/hardware_interface/ContactSensorInterface.h>
// #include <legged_common/hardware_interface/HybridJointInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include "motion_control/common/ModelSettings.h"
#include "motion_control/common/Types.h"
#include "motion_control/gait/MotionPhaseDefinition.h"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


class StateEstimateBase {
 public:
  StateEstimateBase(const rclcpp::Node::SharedPtr& node, ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo info, const ocs2::PinocchioEndEffectorKinematics& eeKinematics);
  virtual void updateJointStates(const ocs2::vector_t& jointPos, const ocs2::vector_t& jointVel);
  virtual void updateContact(contact_flag_t contactFlag) { contactFlag_ = contactFlag; }
  virtual void updateImu(const Eigen::Quaternion<ocs2::scalar_t>& quat, const vector3_t& angularVelLocal, const vector3_t& linearAccelLocal,
                         const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                         const matrix3_t& linearAccelCovariance);

  virtual ocs2::vector_t update(const ocs2::scalar_t& time, const ocs2::scalar_t& period) = 0;

  size_t getMode() { return stanceLeg2ModeNumber(contactFlag_); }

 protected:
  void updateAngular(const vector3_t& zyx, const ocs2::vector_t& angularVel);
  void updateLinear(const ocs2::vector_t& pos, const ocs2::vector_t& linearVel);
  void publishMsgs(const nav_msgs::msg::Odometry& odom);

  ocs2::PinocchioInterface pinocchioInterface_;
  ocs2::CentroidalModelInfo info_;
  std::unique_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematics_;

  vector3_t zyxOffset_ = vector3_t::Zero();
  ocs2::vector_t rbdState_;
  contact_flag_t contactFlag_{};
  Eigen::Quaternion<ocs2::scalar_t> quat_;
  vector3_t angularVelLocal_, linearAccelLocal_;
  matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePub_;

  // rclcpp::Time lastPub_;

};

template <typename T>
T square(T a) {
  return a * a;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}


#include "motion_control/legged_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include "motion_control/common/Types.h"
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

StateEstimateBase::StateEstimateBase(const rclcpp::Node::SharedPtr& node, ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo info,
                                     const ocs2::PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      eeKinematics_(eeKinematics.clone()),
      rbdState_(ocs2::vector_t ::Zero(2 * info_.generalizedCoordinatesNum)),
      node_(node) {

  odomPub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  posePub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
}

void StateEstimateBase::updateJointStates(const ocs2::vector_t& jointPos, const ocs2::vector_t& jointVel) {
  rbdState_.segment(6, info_.actuatedDofNum) = jointPos;
  rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = jointVel;
}

void StateEstimateBase::updateImu(const Eigen::Quaternion<ocs2::scalar_t>& quat, const vector3_t& angularVelLocal,
                                  const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance,
                                  const matrix3_t& angularVelCovariance, const matrix3_t& linearAccelCovariance) {
  quat_ = quat;
  angularVelLocal_ = angularVelLocal;
  linearAccelLocal_ = linearAccelLocal;
  orientationCovariance_ = orientationCovariance;
  angularVelCovariance_ = angularVelCovariance;
  linearAccelCovariance_ = linearAccelCovariance;

  vector3_t zyx = quatToZyx(quat) - zyxOffset_;
  vector3_t angularVelGlobal = ocs2::getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<ocs2::scalar_t>(
      zyx, ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity<ocs2::scalar_t>(quatToZyx(quat), angularVelLocal));
  updateAngular(zyx, angularVelGlobal);
}

void StateEstimateBase::updateAngular(const vector3_t& zyx, const ocs2::vector_t& angularVel) {
  rbdState_.segment<3>(0) = zyx;
  rbdState_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
}

void StateEstimateBase::updateLinear(const ocs2::vector_t& pos, const ocs2::vector_t& linearVel) {
  rbdState_.segment<3>(3) = pos;
  rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
}

void StateEstimateBase::publishMsgs(const nav_msgs::msg::Odometry& odom) {
  // rclcpp::Time time = odom.header.stamp;
  // ocs2::scalar_t publishRate = 200;
  // if ((lastPub_ + rclcpp::Duration::from_seconds(1.0 / publishRate)) < time)
  // {
  //   lastPub_ = time;

  //   odomPub_->publish(odom);

  //   geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  //   pose_msg.header = odom.header;
  //   pose_msg.pose = odom.pose;
  //   posePub_->publish(pose_msg);
  // }
  odomPub_->publish(odom);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header = odom.header;
  pose_msg.pose = odom.pose;
  posePub_->publish(pose_msg);
}


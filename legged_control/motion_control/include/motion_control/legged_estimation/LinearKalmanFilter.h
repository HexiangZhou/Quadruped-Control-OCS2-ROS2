
#pragma once

#include "motion_control/legged_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

// #include <realtime_tools/realtime_buffer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>

class KalmanFilterEstimate : public StateEstimateBase {
 public:
  KalmanFilterEstimate(const rclcpp::Node::SharedPtr& node, ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo info, const ocs2::PinocchioEndEffectorKinematics& eeKinematics);

  ocs2::vector_t update(const ocs2::scalar_t& time, const ocs2::scalar_t& period) override;

  void loadSettings(const std::string& taskFile, bool verbose);

 protected:
  //void updateFromTopic();

  //void callback(const nav_msgs::Odometry::msg::ConstPtr& msg);

  nav_msgs::msg::Odometry getOdomMsg();

  ocs2::vector_t feetHeights_;

  // Config
  ocs2::scalar_t footRadius_ = 0.02;
  ocs2::scalar_t imuProcessNoisePosition_ = 0.02;
  ocs2::scalar_t imuProcessNoiseVelocity_ = 0.02;
  ocs2::scalar_t footProcessNoisePosition_ = 0.002;
  ocs2::scalar_t footSensorNoisePosition_ = 0.005;
  ocs2::scalar_t footSensorNoiseVelocity_ = 0.1;
  ocs2::scalar_t footHeightSensorNoise_ = 0.01;

 private:
  size_t numContacts_, dimContacts_, numState_, numObserve_;

  ocs2::matrix_t a_, b_, c_, q_, p_, r_;
  ocs2::vector_t xHat_, ps_, vs_;

  // Topic
  // ros::Subscriber sub_;
  // realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
  // tf2_ros::Buffer tfBuffer_;
  // tf2_ros::TransformListener tfListener_;
  // tf2::Transform world2odom_;
  // std::string frameOdom_, frameGuess_;
  // bool topicUpdated_;
};


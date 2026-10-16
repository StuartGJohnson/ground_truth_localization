#ifndef GROUND_TRUTH_LOCALIZATION__GROUND_TRUTH_LOCALIZATION_NODE_HPP_
#define GROUND_TRUTH_LOCALIZATION__GROUND_TRUTH_LOCALIZATION_NODE_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace ground_truth_localization {

class GroundTruthLocalizationNode : public rclcpp::Node {
 public:
  GroundTruthLocalizationNode();

 private:
  void on_ground_truth(const nav_msgs::msg::Odometry::SharedPtr input);

  std::string source_topic_;
  std::string odom_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  bool publish_map_to_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr source_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

}  // namespace ground_truth_localization

#endif  // GROUND_TRUTH_LOCALIZATION__GROUND_TRUTH_LOCALIZATION_NODE_HPP_

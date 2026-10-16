#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "ground_truth_localization/ground_truth_localization_node.hpp"

namespace ground_truth_localization {

GroundTruthLocalizationNode::GroundTruthLocalizationNode() : Node("ground_truth_localization_node") {
  source_topic_ = declare_parameter<std::string>("source_topic", "/ground_truth/odom");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
  map_frame_ = declare_parameter<std::string>("map_frame_id", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame_id", "base_link");
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  publish_map_to_odom_ = declare_parameter<bool>("publish_map_to_odom", true);

  if (source_topic_ == odom_topic_ || map_frame_.empty() || odom_frame_.empty() ||
      base_frame_.empty() || map_frame_ == odom_frame_ || odom_frame_ == base_frame_) {
    throw std::invalid_argument("Source/output topics and map/odom/base frames must be distinct and nonempty");
  }

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }
  if (publish_map_to_odom_) {
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped transform;
    // A zero stamp is valid for the latched, time-independent /tf_static transform.
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(transform);
  }

  source_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      source_topic_, rclcpp::QoS(10),
      std::bind(&GroundTruthLocalizationNode::on_ground_truth, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "Republishing %s as %s; TF %s -> %s: %s; static %s -> %s: %s",
              source_topic_.c_str(), odom_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str(),
              publish_tf_ ? "on" : "off", map_frame_.c_str(), odom_frame_.c_str(),
              publish_map_to_odom_ ? "on" : "off");
}

void GroundTruthLocalizationNode::on_ground_truth(const nav_msgs::msg::Odometry::SharedPtr input) {
  // The default identity map->odom transform makes the source map pose
  // numerically valid as an odom pose.
  if (input->header.frame_id != map_frame_ || input->child_frame_id != base_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Skipping ground truth with frames '%s' -> '%s'; expected '%s' -> '%s'",
                         input->header.frame_id.c_str(), input->child_frame_id.c_str(),
                         map_frame_.c_str(), base_frame_.c_str());
    return;
  }

  // PhysX already supplied the exact pose and body-frame twist. Preserve
  // the measurement stamp, both 6x6 covariance arrays, and all state fields.
  auto odom = *input;
  odom.header.frame_id = odom_frame_;
  odom_pub_->publish(odom);

  if (tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }
}

}  // namespace ground_truth_localization

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ground_truth_localization::GroundTruthLocalizationNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("ground_truth_localization_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

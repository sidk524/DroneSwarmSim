#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <functional>
#include <memory>

class PublishDynamicTfNode : public rclcpp::Node
{
public:
  PublishDynamicTfNode() : Node("publish_dynamic_tf_node")
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Use same QoS as publisher (sensor_data profile)
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(
            qos_profile.history,
            qos_profile.depth
        ),
        qos_profile
    );
    
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/px4/odom",
      qos,
      std::bind(&PublishDynamicTfNode::odometry_callback, this, std::placeholders::_1)
    );
  }

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  
  geometry_msgs::msg::TransformStamped odom_to_base_;
  
  void sendTfTransforms();
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

void PublishDynamicTfNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Convert odometry message to TransformStamped
  odom_to_base_.header.stamp = this->get_clock()->now();
  odom_to_base_.header.frame_id = msg->header.frame_id;  // "odom"
  odom_to_base_.child_frame_id = msg->child_frame_id;    // "base_link"
  
  // Copy position to translation
  odom_to_base_.transform.translation.x = msg->pose.pose.position.x;
  odom_to_base_.transform.translation.y = msg->pose.pose.position.y;
  odom_to_base_.transform.translation.z = msg->pose.pose.position.z;
  
  // Copy orientation
  odom_to_base_.transform.rotation.x = msg->pose.pose.orientation.x;
  odom_to_base_.transform.rotation.y = msg->pose.pose.orientation.y;
  odom_to_base_.transform.rotation.z = msg->pose.pose.orientation.z;
  odom_to_base_.transform.rotation.w = msg->pose.pose.orientation.w;
  
  // Publish TF transform immediately when odometry is received
  sendTfTransforms();
}

void PublishDynamicTfNode::sendTfTransforms()
{
  // Only publish if we have received odometry data
  if (!odom_to_base_.header.frame_id.empty())
  {
    tf_broadcaster_->sendTransform(odom_to_base_);
    RCLCPP_DEBUG(this->get_logger(), "Published TF: %s -> %s (pos: %.3f, %.3f, %.3f), timestamp=%d.%09u", 
                 odom_to_base_.header.frame_id.c_str(),
                 odom_to_base_.child_frame_id.c_str(),
                 odom_to_base_.transform.translation.x,
                 odom_to_base_.transform.translation.y,
                 odom_to_base_.transform.translation.z,
                 odom_to_base_.header.stamp.sec,
                 odom_to_base_.header.stamp.nanosec);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishDynamicTfNode>());
  rclcpp::shutdown();
  return 0;
}


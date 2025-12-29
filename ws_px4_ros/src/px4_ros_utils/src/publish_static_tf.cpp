#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class PublishStaticTfNode : public rclcpp::Node
{
public:
  PublishStaticTfNode() : Node("publish_static_tf_node")
  {
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    
    // Send static transforms once during initialization
    sendStaticTfTransforms();
  }

private:
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  
  void sendStaticTfTransforms();
};

void PublishStaticTfNode::sendStaticTfTransforms()
{
  geometry_msgs::msg::TransformStamped base_to_lidar;
  
  base_to_lidar.header.stamp = this->now();
  base_to_lidar.header.frame_id = "base_link";
  base_to_lidar.child_frame_id = "lidar_link";
  
  // Translation: lidar is 0.055m (55mm) above base_link
  base_to_lidar.transform.translation.x = 0.0;
  base_to_lidar.transform.translation.y = 0.0;
  base_to_lidar.transform.translation.z = 0.055;
  
  // Rotation: no rotation (identity quaternion)
  base_to_lidar.transform.rotation.x = 0.0;
  base_to_lidar.transform.rotation.y = 0.0;
  base_to_lidar.transform.rotation.z = 0.0;
  base_to_lidar.transform.rotation.w = 1.0;
  
  static_tf_broadcaster_->sendTransform(base_to_lidar);
  RCLCPP_INFO_ONCE(this->get_logger(), "Published static TF: %s -> %s (translation: %.3f, %.3f, %.3f)", 
                  base_to_lidar.header.frame_id.c_str(),
                  base_to_lidar.child_frame_id.c_str(),
                  base_to_lidar.transform.translation.x,
                  base_to_lidar.transform.translation.y,
                  base_to_lidar.transform.translation.z);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishStaticTfNode>());
  rclcpp::shutdown();
  return 0;
}


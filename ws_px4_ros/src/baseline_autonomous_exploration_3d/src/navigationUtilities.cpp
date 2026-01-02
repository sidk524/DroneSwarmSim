#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <memory>

using namespace std::chrono_literals;
using namespace octomap_msgs::msg;

class NavigationUtilitiesNode : public rclcpp::Node
{
public:
  NavigationUtilitiesNode() : Node("navigation_utilities_node"),
    octomap_subscriber_{this->create_subscription<Octomap>(
      "/octomap_binary",
      10,
      std::bind(&NavigationUtilitiesNode::octomap_callback, this, std::placeholders::_1)
    )}
  {
    RCLCPP_INFO(this->get_logger(), "NavigationUtilitiesNode started");
  }

private:
  void octomap_callback(const Octomap::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received octomap with frame_id: %s", msg->header.frame_id.c_str());
    
    // Deserialize the octomap from the message
    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
    if (tree == nullptr) {
      RCLCPP_WARN(this->get_logger(), "Failed to deserialize octomap from message");
      return;
    }
    
    
    delete tree;
  }

  rclcpp::Subscription<Octomap>::SharedPtr octomap_subscriber_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationUtilitiesNode>());
  rclcpp::shutdown();
  return 0;
}


#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include "px4_msgs/msg/
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>


class FinalPrecisionLand : public px4_ros2::ModeBase
{
    public:
        explicit FinalPrecisionLand(rclcpp::Node & node);
        
    private:
        rclcpp::Subscription<std_msgs::msg::>

}


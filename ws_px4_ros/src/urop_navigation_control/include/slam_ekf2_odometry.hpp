
#include "lifecycle_msgs/srv/change_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include <memory>
#include <octomap/OcTreeNode.h>
#include <px4_ros2/components/mode.hpp>

#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>

#include <px4_ros2/components/mode_executor.hpp>

#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <px4_ros2/odometry/local_position.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>


class SlamEkfOdometry : public rclcpp::Node 
{
    public:
        SlamEkfOdometry();
        
        void slamOdomCallback(nav_msgs::msg::Odometry msg);
        void px4OdomCallback(px4_msgs::msg::VehicleOdometry msg);

    private:
        const rclcpp::QoS qosProfile = rclcpp::QoS(10).reliability_best_available();

        rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odometryPublisher;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slamOdomSubscriber;

        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4OdomSubscriber;

        std::unique_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;

};


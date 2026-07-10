#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rmw/types.h>
#include <rmw/qos_profiles.h>
#include <nav_msgs/msg/odometry.hpp>


class OdomPublisher : public rclcpp::Node
{
    public:
        explicit OdomPublisher();
    
    private:
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometrySubscription;

        std::unique_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;
        std::unique_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
        const rclcpp::QoS qosProfile = rclcpp::QoS(10).reliability_best_available();

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr navOdomPublisher = 
        this->create_publisher<nav_msgs::msg::Odometry>("/nav_msgs/odom", qosProfile);


        void odometryCallback(px4_msgs::msg::VehicleOdometry msg);

};
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rmw/types.h>
#include <rmw/qos_profiles.h>


class OdomPublisher : public rclcpp::Node
{
    public:
        explicit OdomPublisher();
    
    private:
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometrySubscription;   
        std::unique_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;
        const rclcpp::QoS qosProfile = rclcpp::QoS(10).reliability_best_available();

        void odometryCallback(px4_msgs::msg::VehicleOdometry msg);

};
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <memory>
#include <publish_odom_to_base_link.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

OdomPublisher::OdomPublisher() : rclcpp::Node("OdomPublisher"){
        odometrySubscription = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qosProfile, std::bind(
                &OdomPublisher::odometryCallback, this, std::placeholders::_1));
        transformBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

void OdomPublisher::odometeryCallback(px4_msgs::msg::VehicleOdomtery msg){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    tf2::Quaternion q_ned_to_enu;
    tf2::Quaternion q_odom_enu;
    tf2::Quaternion q_odom_ned;

    q_ned_to_enu.setRPY(0, std::numbers::pi, - std::numbers::pi / 2);
    

}
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

void OdomPublisher::odometryCallback(px4_msgs::msg::VehicleOdometry msg){
    RCLCPP_DEBUG(this->get_logger(), "Frame number: %i", msg.pose_frame);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = msg.position[0];
    t.transform.translation.y = msg.position[1];    
    t.transform.translation.z = msg.position[2];
    transformBroadcaster->sendTransform(t);
}


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
}
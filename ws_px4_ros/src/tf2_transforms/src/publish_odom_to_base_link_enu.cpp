#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <memory>
#include <publish_odom_to_base_link.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <cmath>

OdomPublisher::OdomPublisher() : rclcpp::Node("OdomPublisher"){
        odometrySubscription = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qosProfile, std::bind(
                &OdomPublisher::odometryCallback, this, std::placeholders::_1));
        transformBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

void OdomPublisher::odometryCallback(const px4_msgs::msg::VehicleOdometry msg)
{
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    tf2::Quaternion q_ned_frd;
    q_ned_frd.setW(msg.q[0]);
    q_ned_frd.setX(msg.q[1]);
    q_ned_frd.setY(msg.q[2]);
    q_ned_frd.setZ(msg.q[3]);
    q_ned_frd.normalize();

    tf2::Quaternion q_enu_ned;
    q_enu_ned.setRPY(0.0, M_PI, -M_PI / 2.0);
    q_enu_ned.normalize();

    tf2::Quaternion q_frd_flu;
    q_frd_flu.setRPY(M_PI, 0.0, 0.0);
    q_frd_flu.normalize();

    tf2::Quaternion q_enu_flu =
        q_enu_ned * q_ned_frd * q_frd_flu;

    q_enu_flu.normalize();

    t.transform.translation.x = msg.position[1];   // East
    t.transform.translation.y = msg.position[0];   // North
    t.transform.translation.z = -msg.position[2];  // Up

    t.transform.rotation.x = q_enu_flu.x();
    t.transform.rotation.y = q_enu_flu.y();
    t.transform.rotation.z = q_enu_flu.z();
    t.transform.rotation.w = q_enu_flu.w();

    transformBroadcaster->sendTransform(t);
}


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
}

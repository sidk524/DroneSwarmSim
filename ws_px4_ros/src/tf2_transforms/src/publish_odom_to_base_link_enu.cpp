#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <array>
#include <memory>
#include <publish_odom_to_base_link.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <cmath>

OdomPublisher::OdomPublisher() : rclcpp::Node("OdomPublisher"){
        odometrySubscription = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qosProfile, std::bind(
                &OdomPublisher::odometryCallback, this, std::placeholders::_1));

        transformBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
}

void OdomPublisher::odometryCallback(const px4_msgs::msg::VehicleOdometry msg)
{
    geometry_msgs::msg::TransformStamped t;

    nav_msgs::msg::Odometry navOdomMsg; 
 
    t.header.stamp = this->get_clock()->now();

    navOdomMsg.header.stamp = this->get_clock()->now();

    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    navOdomMsg.header.frame_id = "odom";
    navOdomMsg.child_frame_id = "base_link";

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

    navOdomMsg.pose.pose.position.x = msg.position[1];
    navOdomMsg.pose.pose.position.y = msg.position[0];
    navOdomMsg.pose.pose.position.z = -msg.position[2];
    
    std::array<double, 36> covariance = {0.0};

    navOdomMsg.pose.covariance = covariance;

    t.transform.rotation.x = q_enu_flu.x();
    t.transform.rotation.y = q_enu_flu.y();
    t.transform.rotation.z = q_enu_flu.z();
    t.transform.rotation.w = q_enu_flu.w();

    navOdomMsg.pose.pose.orientation.x = q_enu_flu.x();
    navOdomMsg.pose.pose.orientation.y = q_enu_flu.y();
    navOdomMsg.pose.pose.orientation.z = q_enu_flu.z();
    navOdomMsg.pose.pose.orientation.w = q_enu_flu.w();
    
    navOdomMsg.twist.twist.linear.x = msg.velocity[1];
    navOdomMsg.twist.twist.linear.y = msg.velocity[0];
    navOdomMsg.twist.twist.linear.z = -msg.velocity[2];

    navOdomMsg.twist.twist.angular.x = 0.0;
    navOdomMsg.twist.twist.angular.y = 0.0;
    navOdomMsg.twist.twist.angular.z = 0.0;
    try {
        const auto mapToOdom = tfBuffer->lookupTransform("map", "odom", tf2::TimePointZero);
        tf2::doTransform(navOdomMsg.pose.pose, navOdomMsg.pose.pose, mapToOdom);

        tf2::Quaternion qMapOdom;
        tf2::fromMsg(mapToOdom.transform.rotation, qMapOdom);
        const tf2::Vector3 velMap = tf2::quatRotate(qMapOdom,
            {navOdomMsg.twist.twist.linear.x,
             navOdomMsg.twist.twist.linear.y,
             navOdomMsg.twist.twist.linear.z});
        navOdomMsg.twist.twist.linear.x = velMap.x();
        navOdomMsg.twist.twist.linear.y = velMap.y();
        navOdomMsg.twist.twist.linear.z = velMap.z();

        navOdomMsg.header.frame_id = "map";
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "map->odom TF unavailable, publishing odometry in odom frame: %s", ex.what());
    }

    navOdomPublisher->publish(navOdomMsg);

    transformBroadcaster->sendTransform(t);
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
}

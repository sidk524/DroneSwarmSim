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

// void OdomPublisher::odometryCallback(px4_msgs::msg::VehicleOdometry msg){
//     geometry_msgs::msg::TransformStamped t;
//     t.header.stamp = this->get_clock()->now();
//     t.header.frame_id = "odom";
//     t.child_frame_id = "base_link";

//     tf2::Quaternion q_enu_to_ned;
//     tf2::Quaternion q_odom_enu;
//     tf2::Quaternion q_odom_ned;

//     tf2::Vector3 translation_enu;

//     q_enu_to_ned.setRPY(0, M_PI,  -M_PI / 2);
//     q_odom_ned.setW(msg.q[0]);
//     q_odom_ned.setX(msg.q[1]);
//     q_odom_ned.setY(msg.q[2]);
//     q_odom_ned.setZ(msg.q[3]);

//     q_odom_enu = q_enu_to_ned * q_odom_ned * q_enu_to_ned.inverse();

//     translation_enu.setX(msg.position[1]);
//     translation_enu.setY(msg.position[0]);
//     translation_enu.setZ(-1 * msg.position[2]);

//     t.transform.translation.x = translation_enu.getX();
//     t.transform.translation.y = translation_enu.getY();
//     t.transform.translation.z = translation_enu.getZ();

//     t.transform.rotation.w = q_odom_enu.getW();
//     t.transform.rotation.x = q_odom_enu.getX();
//     t.transform.rotation.y = q_odom_enu.getY();
//     t.transform.rotation.z = q_odom_enu.getZ();

//     transformBroadcaster->sendTransform(t);
// }


void OdomPublisher::odometryCallback(const px4_msgs::msg::VehicleOdometry msg)
{
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    /*
        PX4 VehicleOdometry:
        - position is in NED:
            X = North
            Y = East
            Z = Down

        - orientation q is:
            body FRD -> world NED

        ROS wants:
            base_link FLU -> odom ENU

        Therefore:

            q_enu_flu = q_enu_ned * q_ned_frd * q_frd_flu

        where:
            q_enu_ned = world conversion NED -> ENU
            q_ned_frd = PX4 message quaternion
            q_frd_flu = body conversion FLU -> FRD
    */

    // PX4 quaternion: FRD body -> NED world
    tf2::Quaternion q_ned_frd;
    q_ned_frd.setW(msg.q[0]);
    q_ned_frd.setX(msg.q[1]);
    q_ned_frd.setY(msg.q[2]);
    q_ned_frd.setZ(msg.q[3]);
    q_ned_frd.normalize();

    // World-frame conversion: NED -> ENU
    //
    // Converts:
    //   x_enu = y_ned
    //   y_enu = x_ned
    //   z_enu = -z_ned
    tf2::Quaternion q_enu_ned;
    q_enu_ned.setRPY(0.0, M_PI, -M_PI / 2.0);
    q_enu_ned.normalize();

    // Body-frame conversion: FLU -> FRD
    //
    // FLU:
    //   X forward
    //   Y left
    //   Z up
    //
    // FRD:
    //   X forward
    //   Y right
    //   Z down
    //
    // So:
    //   x_frd = x_flu
    //   y_frd = -y_flu
    //   z_frd = -z_flu
    tf2::Quaternion q_frd_flu;
    q_frd_flu.setRPY(M_PI, 0.0, 0.0);
    q_frd_flu.normalize();

    // Final ROS orientation: FLU body -> ENU odom
    tf2::Quaternion q_enu_flu =
        q_enu_ned * q_ned_frd * q_frd_flu;

    q_enu_flu.normalize();

    // Position conversion: NED -> ENU
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

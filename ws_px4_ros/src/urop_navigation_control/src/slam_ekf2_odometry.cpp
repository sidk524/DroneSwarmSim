#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <future>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos_overriding_options.hpp>
#include <rclcpp/utilities.hpp>
#include <slam_ekf2_odometry.hpp>
#include <rclcpp/node.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


SlamEkfOdometry::SlamEkfOdometry() : rclcpp::Node("slam_ekf2"){
    odometryPublisher = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", qosProfile);

    slamOdomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>("/odom", qosProfile, 
        std::bind(&SlamEkfOdometry::slamOdomCallback, this, std::placeholders::_1));

    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    // px4OdomSubscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qosProfile,
    
    // std::bind(&SlamEkfOdometry::px4OdomCallback, this, std::placeholders::_1));

}       


void SlamEkfOdometry::px4OdomCallback(px4_msgs::msg::VehicleOdometry msg){
    RCLCPP_INFO(this->get_logger(), "pos: %f %f %f", msg.position[0], msg.position[1], msg.position[2]);
    RCLCPP_INFO(this->get_logger(), "velocity: %f %f %f", msg.velocity[0], msg.velocity[1], msg.velocity[2]);

}


void SlamEkfOdometry::slamOdomCallback(nav_msgs::msg::Odometry msg){
    px4_msgs::msg::VehicleOdometry px4msg;
    
    px4msg.timestamp_sample =
    static_cast<uint64_t>(msg.header.stamp.sec) * 1'000'000ULL +
    static_cast<uint64_t>(msg.header.stamp.nanosec) / 1'000ULL;

    px4msg.pose_frame = 2;

    tf2::Vector3 pos_tf2;
    tf2::Quaternion q_tf2_result;

    tf2::Quaternion q_tf2(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    
    pos_tf2[0] = msg.pose.pose.position.x;
    pos_tf2[1] = msg.pose.pose.position.y;
    pos_tf2[2] = msg.pose.pose.position.z;

    // try {
    //     const auto odomFromMap = tfBuffer->lookupTransform(
    //         "odom",
    //         msg.header.frame_id,
    //         rclcpp::Time(msg.header.stamp),
    //         rclcpp::Duration::from_seconds(0.05));
    //     tf2::Transform T;
    //     tf2::fromMsg(odomFromMap.transform, T);
    //     pos_tf2 = T * pos_tf2;
    //     q_tf2_result = T.getRotation() * q_tf2;
    //     q_tf2_result.normalize();
    // } catch (const tf2::TransformException & ex) {
    //     RCLCPP_WARN_THROTTLE(
    //         get_logger(),
    //         *get_clock(),
    //         2000,
    //         "Could not transform map pose into odom: %s",
    //         ex.what());

    //         return;
    // }

    Eigen::Matrix<float, 3, 1> pos;
    pos(0) = pos_tf2[0];
    pos(1) = pos_tf2[1];
    pos(2) = pos_tf2[2];

    pos = px4_ros2::positionEnuToNed(pos);

    std::array<float, 3> pos_array;

    pos_array[0] = pos(0,0);
    pos_array[1] = pos(1, 0);
    pos_array[2] = pos(2, 0);

    px4msg.position = pos_array;

    Eigen::Quaternion<float> curr_q(static_cast<float>(q_tf2_result.w()), 
        static_cast<float>(q_tf2_result.x()), 
        static_cast<float>(q_tf2_result.y()), static_cast<float>(q_tf2_result.z()));

    curr_q = px4_ros2::attitudeEnuToNed(curr_q);

    std::array<float, 4> q_array;

    q_array[0] = curr_q.w();
    q_array[1] = curr_q.x();
    q_array[2] = curr_q.y();
    q_array[3] = curr_q.z();

    px4msg.q = q_array;

    px4msg.velocity_frame = 3;

    Eigen::Matrix<float, 3, 1> vel_vector;

    vel_vector(0) = msg.twist.twist.linear.x;
    vel_vector(1) = msg.twist.twist.linear.y;
    vel_vector(2) = msg.twist.twist.linear.z;

    std::array<float, 3> vel_array;

    vel_vector = px4_ros2::fluToFrd(vel_vector);

    vel_array[0] = vel_vector(0,0);
    vel_array[1] = vel_vector(1,0);
    vel_array[2] = vel_vector(2,0);

    px4msg.velocity = vel_array;

    Eigen::Matrix<float, 3, 1> angular_vel_vector;

    angular_vel_vector(0) = msg.twist.twist.angular.x;
    angular_vel_vector(1) = msg.twist.twist.angular.y;
    angular_vel_vector(2) = msg.twist.twist.angular.z;

    std::array<float, 3> angularVel;

    // angular_vel_vector = px4_ros2::fluToFrd(angular_vel_vector);

    angularVel[0] = angular_vel_vector(0,0);
    angularVel[1] = -angular_vel_vector(1,0);
    angularVel[2] = -angular_vel_vector(2,0);

    px4msg.angular_velocity = angularVel;

    Eigen::Matrix<float, 3, 1> posVariance;

    posVariance(0) = msg.pose.covariance[0];
    posVariance(1) = msg.pose.covariance[7];
    posVariance(2) = msg.pose.covariance[14];

    // posVariance = px4_ros2::varianceEnuToNed(posVariance);

    std::array<float, 3> posVarianceArray;

    posVarianceArray[0] = posVariance(0,0);
    posVarianceArray[1] = posVariance(1,0);
    posVarianceArray[2] = posVariance(2,0);


    Eigen::Matrix<float, 3, 1> orientationVariance;

    orientationVariance(0) = msg.pose.covariance[21];
    orientationVariance(1) = msg.pose.covariance[28];
    orientationVariance(2) = msg.pose.covariance[35];

    orientationVariance = px4_ros2::varianceEnuToNed(orientationVariance);

    std::array<float, 3> orientationVarianceArray;

    orientationVarianceArray[0] = orientationVariance(0,0);
    orientationVarianceArray[1] = orientationVariance(1,0);
    orientationVarianceArray[2] = orientationVariance(2,0);

    Eigen::Matrix<float, 3, 1> velocityVariance;

    velocityVariance(0) = msg.twist.covariance[0];
    velocityVariance(1) = msg.twist.covariance[7];
    velocityVariance(2) = msg.twist.covariance[14];

    velocityVariance = px4_ros2::varianceEnuToNed(velocityVariance);

    std::array<float, 3> velocityVarianceArray;

    velocityVarianceArray[0] = velocityVariance(0,0);
    velocityVarianceArray[1] = velocityVariance(1,0);
    velocityVarianceArray[2] = velocityVariance(2,0);

    px4msg.position_variance = posVarianceArray;
    px4msg.orientation_variance = orientationVarianceArray;
    px4msg.velocity_variance = velocityVarianceArray;
    px4msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    odometryPublisher->publish(px4msg);
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamEkfOdometry>());
    rclcpp::shutdown();
}

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
    
    timesyncSubscriber = this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", qosProfile, 
        std::bind(&SlamEkfOdometry::timesyncCallback, this, std::placeholders::_1));


    // px4OdomSubscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qosProfile,
    
    // std::bind(&SlamEkfOdometry::px4OdomCallback, this, std::placeholders::_1));

}       

void SlamEkfOdometry::timesyncCallback(px4_msgs::msg::TimesyncStatus msg) {
    lastOffset = msg.estimated_offset;
}


void SlamEkfOdometry::px4OdomCallback(px4_msgs::msg::VehicleOdometry msg){
    RCLCPP_INFO(this->get_logger(), "pos: %f %f %f", msg.position[0], msg.position[1], msg.position[2]);
    RCLCPP_INFO(this->get_logger(), "velocity: %f %f %f", msg.velocity[0], msg.velocity[1], msg.velocity[2]);

}


void SlamEkfOdometry::slamOdomCallback(nav_msgs::msg::Odometry msg){
    px4_msgs::msg::VehicleOdometry px4msg;
    
    px4msg.timestamp_sample =
    (static_cast<uint64_t>(msg.header.stamp.sec) * 1'000'000ULL +
    static_cast<uint64_t>(msg.header.stamp.nanosec) / 1'000ULL) - lastOffset;

    px4msg.pose_frame = 2;

    Eigen::Matrix<float, 3, 1> pos;
    pos(0) = msg.pose.pose.position.x;
    pos(1) = msg.pose.pose.position.y;
    pos(2) = msg.pose.pose.position.z;

    pos = px4_ros2::positionEnuToNed(pos);

    std::array<float, 3> pos_array;

    pos_array[0] = pos(0,0);
    pos_array[1] = pos(1, 0);
    pos_array[2] = pos(2, 0);

    px4msg.position = pos_array;

    Eigen::Quaternion<float> curr_q(static_cast<float>(msg.pose.pose.orientation.w), 
        static_cast<float>(msg.pose.pose.orientation.x), 
        static_cast<float>(msg.pose.pose.orientation.y), static_cast<float>(msg.pose.pose.orientation.z));

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

    std::array<float, 3> angularVel;

    // angular_vel_vector = px4_ros2::fluToFrd(angular_vel_vector);

    angularVel[0] = msg.twist.twist.angular.x;
    angularVel[1] = -msg.twist.twist.angular.y;
    angularVel[2] = -msg.twist.twist.angular.z;

    px4msg.angular_velocity = angularVel;

    Eigen::Matrix<float, 3, 1> posVariance;

    posVariance(0) = msg.pose.covariance[0];
    posVariance(1) = msg.pose.covariance[7];
    posVariance(2) = msg.pose.covariance[14];

    posVariance = px4_ros2::varianceEnuToNed(posVariance);

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
    px4msg.timestamp = (this->get_clock()->now().nanoseconds() / 1000) - lastOffset;

    odometryPublisher->publish(px4msg);
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamEkfOdometry>());
    rclcpp::shutdown();
}

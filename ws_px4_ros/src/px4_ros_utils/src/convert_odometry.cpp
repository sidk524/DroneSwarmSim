#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <functional>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class ConvertOdometryNode : public rclcpp::Node
{
public:
  ConvertOdometryNode() : Node("convert_odometry_node")
  {
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(
            qos_profile.history,
            qos_profile.depth
        ),
        qos_profile
    );

    vehicle_odometry_subscriber_ = this->create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      qos,
      std::bind(&ConvertOdometryNode::vehicle_odometry_callback, this, std::placeholders::_1)
    );
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/px4/odom", qos);
  }

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  
  void vehicle_odometry_callback(const VehicleOdometry::SharedPtr msg);
};

void ConvertOdometryNode::vehicle_odometry_callback(const VehicleOdometry::SharedPtr msg) {
  nav_msgs::msg::Odometry odom_msg;
  
  // Convert timestamp from microseconds to seconds and nanoseconds
  odom_msg.header.stamp = this->get_clock()->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  
  // Convert position from NED to ENU
  // NED: X=North, Y=East, Z=Down
  // ENU: X=East, Y=North, Z=Up
  odom_msg.pose.pose.position.x = static_cast<double>(msg->position[1]);  // NED Y (East) -> ENU X (East)
  odom_msg.pose.pose.position.y = static_cast<double>(msg->position[0]);  // NED X (North) -> ENU Y (North)
  odom_msg.pose.pose.position.z = -static_cast<double>(msg->position[2]); // NED Z (Down) -> ENU Z (Up)
  
  // Convert quaternion from NED to ENU frame
  // PX4 quaternion is in NED frame (w,x,y,z format)
  // Need to apply NED->ENU rotation: +PI around X, then +PI/2 around Z
  tf2::Quaternion q_ned_frd(msg->q[1], msg->q[2], msg->q[3], msg->q[0]); // if PX4 is [w,x,y,z]
  tf2::Matrix3x3 R_ned_frd(q_ned_frd);
  
  // NED -> ENU basis change
  tf2::Matrix3x3 R_enu_ned(
    0, 1, 0,
    1, 0, 0,
    0, 0,-1
  );
  
  // FRD -> FLU (flip Y and Z)
  tf2::Matrix3x3 R_frd_flu(
    1, 0, 0,
    0,-1, 0,
    0, 0,-1
  );
  
  tf2::Matrix3x3 R_enu_flu = R_enu_ned * R_ned_frd * R_frd_flu;
  
  tf2::Quaternion q_enu_flu;
  R_enu_flu.getRotation(q_enu_flu);
  
  odom_msg.pose.pose.orientation.x = q_enu_flu.x();
  odom_msg.pose.pose.orientation.y = q_enu_flu.y();
  odom_msg.pose.pose.orientation.z = q_enu_flu.z();
  odom_msg.pose.pose.orientation.w = q_enu_flu.w();
  
  
  // Build pose covariance matrix (6x6 = 36 elements, row-major)
  // Order: [x, y, z, roll, pitch, yaw]
  std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
  odom_msg.pose.covariance[0] = static_cast<double>(msg->position_variance[0]);   // x
  odom_msg.pose.covariance[7] = static_cast<double>(msg->position_variance[1]);    // y
  odom_msg.pose.covariance[14] = static_cast<double>(msg->position_variance[2]);   // z
  odom_msg.pose.covariance[21] = static_cast<double>(msg->orientation_variance[0]); // roll
  odom_msg.pose.covariance[28] = static_cast<double>(msg->orientation_variance[1]); // pitch
  odom_msg.pose.covariance[35] = static_cast<double>(msg->orientation_variance[2]); // yaw
  
  // Convert linear velocity from NED to ENU
  odom_msg.twist.twist.linear.x = static_cast<double>(msg->velocity[1]);  // NED Y -> ENU X
  odom_msg.twist.twist.linear.y = static_cast<double>(msg->velocity[0]);  // NED X -> ENU Y
  odom_msg.twist.twist.linear.z = -static_cast<double>(msg->velocity[2]); // NED Z -> ENU Z
  
  // Convert angular velocity from NED to ENU
  odom_msg.twist.twist.angular.x =  msg->angular_velocity[0];
  odom_msg.twist.twist.angular.y = -msg->angular_velocity[1];
  odom_msg.twist.twist.angular.z = -msg->angular_velocity[2];
  
  
  // Build twist covariance matrix (6x6 = 36 elements, row-major)
  // Order: [vx, vy, vz, wx, wy, wz]
  std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
  odom_msg.pose.covariance[0]  = msg->position_variance[1]; // ENU x  <- NED y
  odom_msg.pose.covariance[7]  = msg->position_variance[0]; // ENU y  <- NED x
  odom_msg.pose.covariance[14] = msg->position_variance[2]; // z variance (sign flip doesn’t change variance)
  
  // Angular velocity variance not provided in PX4 message, leave as 0
  
  odometry_publisher_->publish(odom_msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConvertOdometryNode>());
  rclcpp::shutdown();
  return 0;
}


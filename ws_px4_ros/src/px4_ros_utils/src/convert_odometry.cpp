#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
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
  odom_msg.header.stamp.sec = static_cast<int32_t>(msg->timestamp / 1000000);
  odom_msg.header.stamp.nanosec = static_cast<uint32_t>((msg->timestamp % 1000000) * 1000);
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
  tf2::Quaternion q_ned;
  q_ned.setW(static_cast<double>(msg->q[0]));
  q_ned.setX(static_cast<double>(msg->q[1]));
  q_ned.setY(static_cast<double>(msg->q[2]));
  q_ned.setZ(static_cast<double>(msg->q[3]));
  
  // NED to ENU rotation quaternion
  // According to px4_ros_com frame_transforms: quaternion_from_euler(M_PI, 0.0, M_PI/2)
  // This represents: +PI/2 rotation about Z, then +PI rotation around X
  // setRPY(roll, pitch, yaw) applies rotations in order: Z (yaw), Y (pitch), X (roll)
  tf2::Quaternion q_ned_to_enu;
  q_ned_to_enu.setRPY(M_PI, 0.0, M_PI_2);
  
  // Apply rotation: q_enu = q_ned_to_enu * q_ned
  tf2::Quaternion q_enu = q_ned_to_enu * q_ned;
  
  // Convert to ROS format (x, y, z, w)
  odom_msg.pose.pose.orientation.x = q_enu.getX();
  odom_msg.pose.pose.orientation.y = q_enu.getY();
  odom_msg.pose.pose.orientation.z = q_enu.getZ();
  odom_msg.pose.pose.orientation.w = q_enu.getW();
  
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
  odom_msg.twist.twist.angular.x = static_cast<double>(msg->angular_velocity[1]);  // NED Y -> ENU X
  odom_msg.twist.twist.angular.y = static_cast<double>(msg->angular_velocity[0]);  // NED X -> ENU Y
  odom_msg.twist.twist.angular.z = -static_cast<double>(msg->angular_velocity[2]); // NED Z -> ENU Z
  
  // Build twist covariance matrix (6x6 = 36 elements, row-major)
  // Order: [vx, vy, vz, wx, wy, wz]
  std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
  odom_msg.twist.covariance[0] = static_cast<double>(msg->velocity_variance[0]);   // vx
  odom_msg.twist.covariance[7] = static_cast<double>(msg->velocity_variance[1]);   // vy
  odom_msg.twist.covariance[14] = static_cast<double>(msg->velocity_variance[2]);  // vz
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


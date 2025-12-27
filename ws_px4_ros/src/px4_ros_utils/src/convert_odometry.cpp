#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <functional>
#include <algorithm>

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
  
  // Convert position (PX4 uses float32, ROS uses float64)
  odom_msg.pose.pose.position.x = static_cast<double>(msg->position[0]);
  odom_msg.pose.pose.position.y = static_cast<double>(msg->position[1]);
  odom_msg.pose.pose.position.z = static_cast<double>(msg->position[2]);
  
  // Convert quaternion (PX4: w,x,y,z -> ROS: x,y,z,w)
  odom_msg.pose.pose.orientation.x = static_cast<double>(msg->q[1]);
  odom_msg.pose.pose.orientation.y = static_cast<double>(msg->q[2]);
  odom_msg.pose.pose.orientation.z = static_cast<double>(msg->q[3]);
  odom_msg.pose.pose.orientation.w = static_cast<double>(msg->q[0]);
  
  // Build pose covariance matrix (6x6 = 36 elements, row-major)
  // Order: [x, y, z, roll, pitch, yaw]
  std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
  odom_msg.pose.covariance[0] = static_cast<double>(msg->position_variance[0]);   // x
  odom_msg.pose.covariance[7] = static_cast<double>(msg->position_variance[1]);    // y
  odom_msg.pose.covariance[14] = static_cast<double>(msg->position_variance[2]);   // z
  odom_msg.pose.covariance[21] = static_cast<double>(msg->orientation_variance[0]); // roll
  odom_msg.pose.covariance[28] = static_cast<double>(msg->orientation_variance[1]); // pitch
  odom_msg.pose.covariance[35] = static_cast<double>(msg->orientation_variance[2]); // yaw
  
  // Convert linear velocity
  odom_msg.twist.twist.linear.x = static_cast<double>(msg->velocity[0]);
  odom_msg.twist.twist.linear.y = static_cast<double>(msg->velocity[1]);
  odom_msg.twist.twist.linear.z = static_cast<double>(msg->velocity[2]);
  
  // Convert angular velocity
  odom_msg.twist.twist.angular.x = static_cast<double>(msg->angular_velocity[0]);
  odom_msg.twist.twist.angular.y = static_cast<double>(msg->angular_velocity[1]);
  odom_msg.twist.twist.angular.z = static_cast<double>(msg->angular_velocity[2]);
  
  // Build twist covariance matrix (6x6 = 36 elements, row-major)
  // Order: [vx, vy, vz, wx, wy, wz]
  std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
  odom_msg.twist.covariance[0] = static_cast<double>(msg->velocity_variance[0]);   // vx
  odom_msg.twist.covariance[7] = static_cast<double>(msg->velocity_variance[1]);   // vy
  odom_msg.twist.covariance[14] = static_cast<double>(msg->velocity_variance[2]);  // vz
  // Angular velocity variance not provided in PX4 message, leave as 0
  
  // Publish the converted message
  odometry_publisher_->publish(odom_msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConvertOdometryNode>());
  rclcpp::shutdown();
  return 0;
}


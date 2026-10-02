#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_ros_com/frame_transforms.h> // Provides the ned_to_enu conversion tools

class Px4ToRvizOdom : public rclcpp::Node {
public:
    Px4ToRvizOdom() : Node("px4_to_rviz_odom") {
        // Best effort QoS configuration is required to match PX4's publishing profile
        auto qos_profile = rclcpp::SensorDataQoS();

        // Subscribe to the raw PX4 odometry topic coming from the Micro-XRCE Agent
        px4_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 
            qos_profile, 
            std::bind(&Px4ToRvizOdom::odom_callback, this, std::placeholders::_1));

        // Publish a standard ROS 2 message that RViz2 natively accepts
        rviz_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/rviz/vehicle_odometry", 10);

        RCLCPP_INFO(this->get_logger(), "PX4 to RViz Odometry Converter Node has started.");
    }

private:
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        auto rviz_msg = nav_msgs::msg::Odometry();

        // 1. Set standard header properties
        rviz_msg.header.stamp = this->get_clock()->now();
        rviz_msg.header.frame_id = "map";         // Fixed frame in RViz
        rviz_msg.child_frame_id = "base_link";   // Dynamic frame representing the drone

        // 2. Position Transformation: Convert from PX4 (NED) to ROS 2 (ENU)
        Eigen::Vector3d px4_pos(msg->position[0], msg->position[1], msg->position[2]);
        Eigen::Vector3d enu_pos = px4_ros_com::frame_transforms::ned_to_enu_local_frame(px4_pos);
        
        rviz_msg.pose.pose.position.x = enu_pos.x();
        rviz_msg.pose.pose.position.y = enu_pos.y();
        rviz_msg.pose.pose.position.z = enu_pos.z();

        // 3. Orientation Transformation: Convert Quaternion from PX4 (NED/FRD) to ROS 2 (ENU/FLU)
        Eigen::Quaterniond px4_q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        Eigen::Quaterniond enu_q = px4_ros_com::frame_transforms::ned_to_enu_orientation(px4_q);

        rviz_msg.pose.pose.orientation.x = enu_q.x();
        rviz_msg.pose.pose.orientation.y = enu_q.y();
        rviz_msg.pose.pose.orientation.z = enu_q.z();
        rviz_msg.pose.pose.orientation.w = enu_q.w();

        // 4. Linear Velocity Transformation: Convert from PX4 (NED) to ROS 2 (ENU)
        Eigen::Vector3d px4_vel(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
        Eigen::Vector3d enu_vel = px4_ros_com::frame_transforms::ned_to_enu_local_frame(px4_vel);

        rviz_msg.twist.twist.linear.x = enu_vel.x();
        rviz_msg.twist.twist.linear.y = enu_vel.y();
        rviz_msg.twist.twist.linear.z = enu_vel.z();

        // 5. Publish the formatted message to RViz
        rviz_pub_->publish(rviz_msg);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rviz_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4ToRvizOdom>());
    rclcpp::shutdown();
    return 0;
}

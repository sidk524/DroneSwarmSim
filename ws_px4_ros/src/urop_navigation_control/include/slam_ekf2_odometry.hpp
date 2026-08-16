
#include "lifecycle_msgs/srv/change_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <std_msgs/msg/float64.hpp>
#include "quadrotor_msgs/msg/position_command.hpp"
#include <memory>
#include <octomap/OcTreeNode.h>
#include <px4_ros2/components/mode.hpp>

#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>

#include <px4_ros2/components/mode_executor.hpp>

#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <px4_ros2/odometry/local_position.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>


class SlamEkfOdometry : public rclcpp::Node 
{
    public:
        SlamEkfOdometry();
        
        void slamOdomCallback(nav_msgs::msg::Odometry msg);
        void px4OdomCallback(px4_msgs::msg::VehicleOdometry msg);
        void timesyncCallback(px4_msgs::msg::TimesyncStatus msg);
        void groundTruthCallback(geometry_msgs::msg::PoseArray msg);

    private:
        const rclcpp::QoS qosProfile = rclcpp::QoS(10).reliability_best_available();

        std::array<double, 3> lastSlamOdomPos;
        std::array<double, 3> lastPx4OdomPos;
        std::array<double, 3> groundTruthDronePose;

        std::array<double, 3> poseOffset;

        bool groundTruthReceived = false;
        bool px4OdomReceived;

        rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odometryPublisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pythagPublisher;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slamOdomSubscriber;

        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4OdomSubscriber;
        rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesyncSubscriber;

        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr groundTruthPoseSubscriber;




        std::int64_t lastOffset = 0;

        std::unique_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;

};



#include "lifecycle_msgs/srv/change_state.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include <memory>
#include <octomap/OcTreeNode.h>
#include <px4_ros2/components/mode.hpp>

#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>

#include <px4_ros2/components/mode_executor.hpp>

#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <px4_ros2/odometry/local_position.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <std_msgs/msg/bool.hpp>


class AutonomousNavigationMode : public px4_ros2::ModeBase {
    public:
        explicit AutonomousNavigationMode(rclcpp::Node & node);
        void onActivate() override;
        void onDeactivate() override;


        rclcpp::Node &_node;

        std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectorySetpoint;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypointPublisher;

        void sendWaypoint(std_msgs::msg::Bool msg1);
        void positionCmdCallback(quadrotor_msgs::msg::PositionCommand msg);
        px4_ros2::TrajectorySetpoint coords;


    private:
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeStateService;
        std::unique_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
        const rclcpp::QoS qosProfile = rclcpp::QoS(10).reliability_best_available();

        bool readyForWaypoint = false;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr readyForWaypointSub;

        rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr positionCmdSubscriber;

        tf2::Vector3 pos;


};
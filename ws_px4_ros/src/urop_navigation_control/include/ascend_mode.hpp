
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
#include <px4_ros2/odometry/odometry.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <rclcpp/timer.hpp>
#include <px4_ros2/utils/subscription.hpp>




class AscendMode : public px4_ros2::ModeBase {
    public:
        explicit AscendMode(rclcpp::Node & node);
        void onActivate() override;
        void onDeactivate() override;
        rclcpp::Node &_node;
        std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectorySetpoint;
        
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odomSubscriber;

        rclcpp::TimerBase::SharedPtr wall_timer;
        void ascend();
        void checkAscended(px4_msgs::msg::VehicleOdometry msg);

    private:       
        const rclcpp::QoS qosProfile = rclcpp::QoS(10).reliability_best_available();

};
#include "lifecycle_msgs/srv/change_state.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include <autonomous_navigation_mode.hpp>
#include <memory>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <string>
#include <cmath>


AutonomousNavigationMode::AutonomousNavigationMode(rclcpp::Node & node) : px4_ros2::ModeBase(node, Settings("Auto navigation mode")),
    _node(node)
    {
        trajectorySetpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        positionCmdSubscriber = _node.create_subscription<quadrotor_msgs::msg::PositionCommand>("/position_cmd", qosProfile, 
        std::bind(&AutonomousNavigationMode::positionCmdCallback, this, std::placeholders::_1));
        changeStateService = _node.create_client<lifecycle_msgs::srv::ChangeState>("/lifecycle_navigation_node/change_state");

        tfBuffer = std::make_unique<tf2_ros::Buffer>(_node.get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    }

void AutonomousNavigationMode::positionCmdCallback(quadrotor_msgs::msg::PositionCommand msg) {
    tf2::Vector3 pos(msg.position.x, msg.position.y, msg.position.z);
    tf2::Vector3 vel(msg.velocity.x, msg.velocity.y, msg.velocity.z);
    tf2::Vector3 acc(msg.acceleration.x, msg.acceleration.y, msg.acceleration.z);
    double yaw = msg.yaw;

    // the planner works in the map frame, but px4's local frame corresponds to
    // odom, so re-express the command in odom before the ENU->NED conversion
    try {
        const auto odomFromMap = tfBuffer->lookupTransform("odom", "map", tf2::TimePointZero);
        tf2::Transform T;
        tf2::fromMsg(odomFromMap.transform, T);

        pos = T * pos;
        vel = T.getBasis() * vel;
        acc = T.getBasis() * acc;
        yaw += tf2::getYaw(odomFromMap.transform.rotation);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 5000,
            "odom->map TF unavailable, passing command through unchanged: %s", ex.what());
    }

    coords = {};
    coords.withPositionX(pos.y()).withPositionY(pos.x()).withPositionZ(-pos.z()).withYaw(M_PI/2 - yaw)
    .withVelocityX(vel.y()).withVelocityY(vel.x()).withVelocityZ(-vel.z())
    .withAccelerationX(acc.y()).withAccelerationY(acc.x()).withAccelerationZ(-acc.z())
    .withYawRate(-msg.yaw_dot);

    trajectorySetpoint->update(coords);
}

void AutonomousNavigationMode::onActivate(){
    RCLCPP_INFO(_node.get_logger(), "activated auto navigation node");

    auto requestToInactive = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    requestToInactive->transition.id = 1;
    requestToInactive->transition.label = "configure";

    using ServiceResponseFuture = rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture;

    std::function<void(ServiceResponseFuture)> handle_response_1 = [this](ServiceResponseFuture future) {
        auto result = future.get(); // Retrieves the processed data structure
        RCLCPP_INFO(_node.get_logger(), "unconfigure to inactive status: %d", result->success);

        if(result->success){
            auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

            request->transition.id = 3;
            request->transition.label = "activate";


            auto handle_response = [this](ServiceResponseFuture future) {
                auto result = future.get(); // Retrieves the processed data structure
                RCLCPP_INFO(_node.get_logger(), "inactive to active status: %d", result->success);

            };

            changeStateService->async_send_request(request, handle_response);

        }
    };

        
    changeStateService->async_send_request(requestToInactive, handle_response_1);


    RCLCPP_INFO(_node.get_logger(), "sent configuring -> inactive transition");


}


void AutonomousNavigationMode::onDeactivate(){
}



int main(int argc, char *argv[]){
    using autoNavigationMode = px4_ros2::NodeWithMode<AutonomousNavigationMode>;

    static const std::string kNodeName = "auto_navigation_node";
    static const bool kEnableDebugOutput = true;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<autoNavigationMode>(kNodeName, kEnableDebugOutput));
    rclcpp::shutdown();

}
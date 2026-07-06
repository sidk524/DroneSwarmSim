#include "lifecycle_msgs/srv/change_state.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include <autonomous_navigation_mode.hpp>
#include <memory>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
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
    }

void AutonomousNavigationMode::positionCmdCallback(quadrotor_msgs::msg::PositionCommand msg) {
    coords = {};
    coords.withPositionX(msg.position.y).withPositionY(msg.position.x).withPositionZ(-msg.position.z).withYaw(M_PI/2 - msg.yaw );
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
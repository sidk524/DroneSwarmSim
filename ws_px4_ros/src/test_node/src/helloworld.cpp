#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include <memory>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <rclcpp/create_client.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <stdint.h>

#include <string>
#include <vector>

using namespace std;
using namespace px4_msgs::msg;

class TestNode : public rclcpp::Node
{
public:
    TestNode(string px4_namespace):
        Node("TestNode"),
        currState{State::init},
        service_done{false},
        service_result{0},
        numSteps{0},
        offboardPublisher{this->create_publisher<OffboardControlMode>(px4_namespace + "in/offboard_control_mode", 10)},
        trajectoryPublisher{this->create_publisher<TrajectorySetpoint>(px4_namespace + "in/trajectory_setpoint", 10)},
        vehicleCommandClient{this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace + "vehicle_command")}
        {
            RCLCPP_INFO(this->get_logger(), "Starting Offboard Control example with PX4 services");
            RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
            while (!vehicleCommandClient->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            RCLCPP_INFO(this->get_logger(), "starting timer callback");

            timer = this->create_wall_timer(100ms, std::bind(&TestNode::timer_callback, this));
        }

    void arm();
    void disarm();

private:
    enum class State{
        init,
        offboard_requested,
        wait_for_stable_offboard_mode,
        arm_requested
    };

    struct NEDposition {
        float pos[3];
    };

    State currState;
    bool service_done;
    uint8_t service_result;
    int numSteps;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboardPublisher;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectoryPublisher;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicleCommandClient;
    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicleGlobalPositionSubscriber;
    rclcpp::TimerBase::SharedPtr timer;
    
    void timer_callback(void);
    void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void client_response(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    void switch_to_offboard_mode();
    void publish_offboard_mode();
    void publish_trajectory_setpoint();
    void request_current_coords();
};

void TestNode::request_vehicle_command(uint16_t command, float param1, float param2){

    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
    request->request = msg;
    service_done = false;
    auto result = vehicleCommandClient->async_send_request(request, 
        std::bind(&TestNode::client_response, this, std::placeholders::_1));
};

void TestNode::client_response(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future){
    RCLCPP_INFO(this->get_logger(), "received client response");

    auto status = future.wait_for(1s);
    if (status == std::future_status::ready){

        auto result = future.get()->reply;
        service_result = result.result;
        service_done = true;
    }
}

void TestNode::switch_to_offboard_mode(){
    RCLCPP_INFO(this->get_logger(), "requesting switch to offboard mode");

    request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
};

void TestNode::publish_offboard_mode(){

    px4_msgs::msg::OffboardControlMode msg {};
    msg.position = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    msg.velocity = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboardPublisher->publish(msg);
}

void TestNode::publish_trajectory_setpoint(){
    px4_msgs::msg::TrajectorySetpoint msg {};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = {0.0, 0.0, -20.0};
    msg.velocity = {0.01, 0.01, 0.01};
    trajectoryPublisher->publish(msg);  
}

void TestNode::arm(){
                RCLCPP_INFO(this->get_logger(), "requesting arm");

    request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}


void TestNode::disarm(){
                    RCLCPP_INFO(this->get_logger(), "requesting disarm");

    request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}


void TestNode::timer_callback(void){
    publish_offboard_mode();
    publish_trajectory_setpoint();

    
    switch(currState){
        case State::init:
            switch_to_offboard_mode();
            currState = State::offboard_requested;
        case State::offboard_requested:
            if (service_done){
                if (service_result == 0){
                    currState = State::wait_for_stable_offboard_mode;
                } else {
                    RCLCPP_INFO(this->get_logger(), "shutting down");

                    rclcpp::shutdown();
                }
            }
        case State::wait_for_stable_offboard_mode:
            if (++numSteps < 11){
                break;
            } else{
                arm();
                currState = State::arm_requested;
            }
        case State::arm_requested:
            if (service_done){
                if (service_result == 0){
                    currState = State::flying_to_height;
                } else{
                    RCLCPP_INFO(this->get_logger(), "shutting down");

                    rclcpp::shutdown();
                }
            }
            break;
        default:
            break;
        }
       
};


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TestNode>("/fmu/"));
	rclcpp::shutdown();
	return 0;
}



#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include <cmath>
#include <memory>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>

#include <rclcpp/create_client.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rmw/types.h>
#include <stdint.h>

#include <rmw/qos_profiles.h>


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
        ratesPublisher{this->create_publisher<VehicleRatesSetpoint>(px4_namespace + "in/vehicle_rates_setpoint", 10)},
        vehicleCommandClient{this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace + "vehicle_command")},
        positionSubscription{this->create_subscription<VehicleLocalPosition>(
            px4_namespace + "out/vehicle_local_position_v1", rclcpp::QoS(10).best_effort(), bind(&TestNode::topic_callback, this, placeholders::_1))}
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
        arm_requested,
        flying,
        request_orbit,
        orbit
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
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr ratesPublisher;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicleCommandClient;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr positionSubscription;
    rclcpp::TimerBase::SharedPtr timer;
    
    void topic_callback(const VehicleLocalPosition::SharedPtr msg);
    void timer_callback(void);
    void request_vehicle_command(uint16_t command, float param1 = 0.0, 
            float param2 = 0.0,
            float param3 = 0.0,
            float param4 = 0.0,
            double param5 = 0.0,
            double param6 = 0.0,
            float param7 = 0.0
        );
    void client_response(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    void switch_to_offboard_mode();
    void publish_offboard_mode(bool position, bool velocity, bool attitude, bool body_rate);
    void publish_trajectory_setpoint();
    void set_orbit(float radius, float velocity);

};

void TestNode::topic_callback(const VehicleLocalPosition::SharedPtr msg) {
    if (currState == State::flying){
        if (msg->z <= -19.5 && msg->z >= -20.5){
            currState = State::request_orbit;
            set_orbit(30.0, 100.0);
        }
    }
}

void TestNode::request_vehicle_command(uint16_t command, 
    float param1, 
    float param2,
    float param3,
    float param4,
    double param5,
    double param6,
    float param7
){
    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;

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

    auto status = future.wait_for(1s);
    if (status == std::future_status::ready){

        auto result = future.get()->reply;

        service_result = result.result;
        service_done = true;
        RCLCPP_INFO(this->get_logger(), "received client response");

    }
}


void TestNode::switch_to_offboard_mode(){
    RCLCPP_INFO(this->get_logger(), "requesting switch to offboard mode");
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

};

void TestNode::set_orbit(float radius, float velocity){
    RCLCPP_INFO(this->get_logger(), "requesting orbit");
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_ORBIT, radius, velocity,
    VehicleCommand::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER,
    0.0,
    NAN,
    NAN,
    NAN
);
}

void TestNode::publish_offboard_mode(bool position = true, bool velocity = false, bool attitude = false, bool body_rate = false){
    px4_msgs::msg::OffboardControlMode msg {};
    msg.position = position;
    msg.acceleration = false;
    msg.attitude = attitude;
    msg.body_rate = body_rate;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    msg.velocity = velocity;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboardPublisher->publish(msg);
}

void TestNode::publish_trajectory_setpoint(){
    px4_msgs::msg::TrajectorySetpoint msg {};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = {0.0, 0.0, -20.0};
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
    switch(currState){
        case State::init:
            publish_offboard_mode();
            switch_to_offboard_mode();
            currState = State::offboard_requested;
        case State::offboard_requested:
            publish_offboard_mode();

            if (service_done){
                if (service_result == 0){
                    currState = State::wait_for_stable_offboard_mode;
                } else {
                    RCLCPP_INFO(this->get_logger(), "shutting down");

                    rclcpp::shutdown();
                }
            }
        case State::wait_for_stable_offboard_mode:
            publish_offboard_mode();

            if (++numSteps < 11){
                break;
            } else{
                arm();
                currState = State::arm_requested;
            }
        case State::arm_requested:
            publish_offboard_mode();

            if (service_done){
                if (service_result == 0){
                    currState = State::flying;
                } else{
                    RCLCPP_INFO(this->get_logger(), "shutting down");
                    rclcpp::shutdown();
                }
            }
            break;
        case State::flying:
            //RCLCPP_INFO(this->get_logger(), "state: flying");

            publish_offboard_mode();
            publish_trajectory_setpoint();
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



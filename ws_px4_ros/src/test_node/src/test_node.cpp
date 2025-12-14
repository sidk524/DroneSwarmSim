#include <array>
#include <px4_msgs/msg/detail/vehicle_command__struct.hpp>
#include <px4_msgs/msg/detail/vehicle_local_position__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

using namespace px4_msgs::msg;
using namespace std::chrono_literals;

class TestNode : public rclcpp::Node
{
public:
  TestNode() : Node("test_node")
  {
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    

    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(
            qos_profile.history,
            qos_profile.depth
        ),
        qos_profile
    );

    auto position_callback = [this](const VehicleLocalPosition & msg) -> void {
      std::cout << "Current Position x:" << 
      std::to_string(msg.x) << " y: " 
      << std::to_string(msg.y) <<
        " z: " << std::to_string(msg.z) << std::endl;

      if (std::abs(msg.x - 0.0) < 0.1 && std::abs(msg.y - 0.0) < 0.1 && std::abs(msg.z - -5.0) < 0.1){
        this->currentPosition = {5,0,-5};
      }
    };

    auto timer_callback = [this]() -> void {
			if (offboard_setpoint_counter_ == 10) {
				this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->armDrone(); 
        std::cout << "Armed Drone" << std::endl;
			}

			publishOffboardControlMode();
			publishNewPosition();

			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
    vehicle_position_subscriber_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1", qos, position_callback);
    
		timer_ = this->create_wall_timer(100ms, timer_callback);

  }

  void armDrone();
  void disarmDrone();
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

  std::array<float, 3> currentPosition = {0.0, 0.0, -5.0};
  
private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Tick");
  }
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_position_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  int offboard_setpoint_counter_ = 0;

  void publishNewPosition();
  void publishOffboardControlMode();
  void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void TestNode::publishNewPosition() {
  TrajectorySetpoint msg{};
  msg.position = this->currentPosition;
  msg.yaw = -3.14;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
  std::cout << "Published position" << std::endl;
}

void TestNode::publishOffboardControlMode(){
  OffboardControlMode msg{};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;
  offboard_control_mode_publisher_->publish(msg);
}

void TestNode::publishVehicleCommand(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

void TestNode::armDrone() {
  publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

void TestNode::disarmDrone() {
  publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}

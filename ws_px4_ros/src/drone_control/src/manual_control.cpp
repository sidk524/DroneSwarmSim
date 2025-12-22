#include <array>
#include <algorithm>
#include <functional>
#include <px4_msgs/msg/detail/vehicle_command__struct.hpp>
#include <px4_msgs/msg/detail/vehicle_local_position__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <X11/Xlib.h>
#include <X11/keysym.h>

using namespace px4_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std::chrono_literals;

struct KeyStates {
  bool key_w;
  bool key_a;
  bool key_s;
  bool key_d;
  bool key_q;
  bool key_e;
  bool key_c;
  bool key_ctrl;
};

class ManualControlNode : public rclcpp::Node
{
public:
  ManualControlNode() : Node("manual_control_node")
  {
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    manual_control_input_publisher_ = this->create_publisher<ManualControlSetpoint>("/fmu/in/manual_control_input", 10);
    lidar_data_subscriber_ = this->create_subscription<LaserScan>(
      "/lidar/scan", 
      10, 
      std::bind(&ManualControlNode::lidar_data_callback, this, std::placeholders::_1)
    );
    
    last_time = this->now();

    auto timer_callback = [this]() -> void {
			if (offboard_setpoint_counter_ == 10) {
				this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);
				this->armDrone(); 
        std::cout << "Armed Drone" << std::endl;
			}
			updateControlValues();
			publishManualInput();

			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
    
		timer_ = this->create_wall_timer(20ms, timer_callback);

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
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<ManualControlSetpoint>::SharedPtr manual_control_input_publisher_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_position_subscriber_;
  rclcpp::Subscription<LaserScan>::SharedPtr lidar_data_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  int offboard_setpoint_counter_ = 0;
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float throttle = -1.0f;
  rclcpp::Time last_time;

  const float STICK_RATE     = 1.5f;  // units / second
  const float RETURN_RATE    = 3.0f;  // spring back strength
  const float THROTTLE_RATE  = 0.6f;  // thrust change / second
  const float MAX_TILT       = 0.4f;  // limit roll/pitch magnitude
  
  bool isKeyDown(Display* display, const char* keys, KeySym keysym);
  KeyStates getKeysPressed();
  void updateControlValues();
  void publishManualInput();
  void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
  void lidar_data_callback(const LaserScan::SharedPtr msg);
};

void ManualControlNode::publishVehicleCommand(uint16_t command, float param1, float param2)
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

void ManualControlNode::armDrone() {
  publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

void ManualControlNode::disarmDrone() {
  publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

void ManualControlNode::lidar_data_callback(const LaserScan::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "=== LaserScan Data ===");
  RCLCPP_INFO(this->get_logger(), "Header:");
  RCLCPP_INFO(this->get_logger(), "  timestamp: %d.%09u", msg->header.stamp.sec, msg->header.stamp.nanosec);
  RCLCPP_INFO(this->get_logger(), "  frame_id: %s", msg->header.frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "Angle: min=%.3f, max=%.3f, increment=%.3f [rad]", 
              msg->angle_min, msg->angle_max, msg->angle_increment);
  RCLCPP_INFO(this->get_logger(), "Time: increment=%.6f [s], scan_time=%.6f [s]", 
              msg->time_increment, msg->scan_time);
  RCLCPP_INFO(this->get_logger(), "Range: min=%.3f, max=%.3f [m]", 
              msg->range_min, msg->range_max);
  RCLCPP_INFO(this->get_logger(), "Ranges: %zu measurements", msg->ranges.size());
  RCLCPP_INFO(this->get_logger(), "Intensities: %zu measurements", msg->intensities.size());
  
  if (msg->ranges.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "First few ranges: %.3f, %.3f, %.3f, %.3f, %.3f [m]",
                msg->ranges[0],
                msg->ranges.size() > 1 ? msg->ranges[1] : 0.0f,
                msg->ranges.size() > 2 ? msg->ranges[2] : 0.0f,
                msg->ranges.size() > 3 ? msg->ranges[3] : 0.0f,
                msg->ranges.size() > 4 ? msg->ranges[4] : 0.0f);
  }
}

bool ManualControlNode::isKeyDown(Display* display, const char* keys, KeySym keysym) {
  KeyCode keycode = XKeysymToKeycode(display, keysym);
  if (keycode == 0) {
    return false;
  }
  
  int byte_index = keycode / 8;
  int bit_index = keycode % 8;
  return (byte_index < 32 && (keys[byte_index] & (1 << bit_index)));
}

KeyStates ManualControlNode::getKeysPressed() {
  KeyStates states = {false, false, false, false, false, false, false, false};
  
  Display* display = XOpenDisplay(NULL);
  if (display == NULL) {
    return states;
  }
  
  char keys[32];
  XQueryKeymap(display, keys);
  
  states.key_w = isKeyDown(display, keys, XK_w);
  states.key_a = isKeyDown(display, keys, XK_a);
  states.key_s = isKeyDown(display, keys, XK_s);
  states.key_d = isKeyDown(display, keys, XK_d);
  states.key_q = isKeyDown(display, keys, XK_q);
  states.key_e = isKeyDown(display, keys, XK_e);
  states.key_c = isKeyDown(display, keys, XK_c);
  states.key_ctrl = isKeyDown(display, keys, XK_Control_L) || isKeyDown(display, keys, XK_Control_R);
  
  XCloseDisplay(display);
  
  return states;
}

void ManualControlNode::updateControlValues() {
  // Timing (dt)
  rclcpp::Time now = this->now();
  float dt = (now - last_time).seconds();
  last_time = now;
  
  // Safety clamp
  dt = std::min(dt, 0.05f);
  
  // Get keyboard state
  KeyStates keys = getKeysPressed();
  
  // Roll (A / D)
  if (keys.key_d) roll += STICK_RATE * dt;
  if (keys.key_a) roll -= STICK_RATE * dt;
  
  if (!keys.key_a && !keys.key_d) {
    roll -= roll * RETURN_RATE * dt;
  }
  
  roll = std::clamp(roll, -MAX_TILT, MAX_TILT);
  
  // Pitch (W / S)
  if (keys.key_w) pitch += STICK_RATE * dt;
  if (keys.key_s) pitch -= STICK_RATE * dt;
  
  if (!keys.key_w && !keys.key_s) {
    pitch -= pitch * RETURN_RATE * dt;
  }
  
  pitch = std::clamp(pitch, -MAX_TILT, MAX_TILT);
  
  // Yaw (Q / E)
  if (keys.key_e) yaw += STICK_RATE * dt;
  if (keys.key_q) yaw -= STICK_RATE * dt;
  
  if (!keys.key_q && !keys.key_e) {
    yaw -= yaw * RETURN_RATE * dt;
  }
  
  yaw = std::clamp(yaw, -1.0f, 1.0f);
  
  // Throttle (C / Ctrl)
  if (keys.key_c) throttle += THROTTLE_RATE * dt;
  if (keys.key_ctrl) throttle -= THROTTLE_RATE * dt;

  if (!keys.key_c && !keys.key_ctrl) {
    throttle -= throttle * RETURN_RATE * dt;
  }
  
  throttle = std::clamp(throttle, -1.0f, 1.0f);
}

void ManualControlNode::publishManualInput() {
  ManualControlSetpoint msg{};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg.timestamp_sample = msg.timestamp;
  msg.valid = true;
  msg.data_source = ManualControlSetpoint::SOURCE_MAVLINK_0;
  
  msg.roll = roll;
  msg.pitch = pitch;    
  msg.yaw = yaw;      
  msg.throttle = throttle;  
  
  // Check if any keys are currently pressed
  KeyStates keys = getKeysPressed();
  msg.sticks_moving = keys.key_w || keys.key_a || keys.key_s || keys.key_d || 
                      keys.key_q || keys.key_e || keys.key_c || keys.key_ctrl;
  msg.buttons = 0;
  
  manual_control_input_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualControlNode>());
  rclcpp::shutdown();
  return 0;
}

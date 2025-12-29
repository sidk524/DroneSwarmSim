#include <array>
#include <algorithm>
#include <functional>
#include <px4_msgs/msg/detail/vehicle_command__struct.hpp>
#include <px4_msgs/msg/detail/vehicle_local_position__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
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
  ManualControlNode() : Node("manual_control_node"),
  vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command")},
  manual_control_input_publisher_{this->create_publisher<ManualControlSetpoint>("/fmu/in/manual_control_input", 10)},
  lidar_data_subscriber_{this->create_subscription<LaserScan>(
    "/lidar/scan", 
    10, 
    std::bind(&ManualControlNode::lidar_data_callback, this, std::placeholders::_1)
  )},
  state_{State::init},
  service_result_{0},
  service_done_{false},
  arm_retry_count_{0}
  {
    while (!vehicle_command_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        return;
      }
    }

    last_time = this->now();

    auto timer_callback = [this]() -> void {
      updateControlValues();
      publishManualInput();
      switch (state_) {
        case State::init:
          this->requestVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);
          state_ = State::position_mode_requested;
          break;
        case State::position_mode_requested:
          if (service_done_) {
            if (service_result_ == 0) {
              state_ = State::wait_for_stable_position_mode;
            }
            else {
              rclcpp::shutdown();
            }
          }
          break;
        case State::wait_for_stable_position_mode:
          // If retrying, wait 1 second before attempting again
          if (arm_retry_count_ > 0) {
            if (this->now() >= arm_retry_time_) {
              RCLCPP_INFO(this->get_logger(), "Retrying to arm drone...");
              armDrone();
              state_ = State::arm_requested;
            }
          } else {
            // First attempt - arm immediately
            armDrone();
            state_ = State::arm_requested;
          }
          break;
        case State::arm_requested:
          if (service_done_) {
            if (service_result_ == 0) {
              state_ = State::armed;
              arm_retry_count_ = 0;
            }
            else {
              arm_retry_count_++;
              RCLCPP_WARN(
                this->get_logger(),
                "Failed to arm drone (%s, result=%u), attempt %d/3",
                vehicle_cmd_result_to_string(service_result_),
                static_cast<unsigned>(service_result_),
                arm_retry_count_);
              
              if (arm_retry_count_ >= 50) {
                RCLCPP_ERROR(
                  this->get_logger(),
                  "Failed to arm drone after 3 attempts, exiting");
                rclcpp::shutdown();
              } else {
                // Wait 1 second before retrying - transition back to wait state
                arm_retry_time_ = this->now() + 1s;
                state_ = State::wait_for_stable_position_mode;
              }
            }
          }
          break;
        case State::armed:
          
          break;
      }
		};
    
		timer_ = this->create_wall_timer(20ms, timer_callback);

  }

  void armDrone();
  void disarmDrone();
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

  std::array<float, 3> currentPosition = {0.0, 0.0, -5.0};
  
private:
  static const char *vehicle_cmd_result_to_string(uint8_t result)
  {
    using px4_msgs::msg::VehicleCommandAck;
    switch (result) {
      case VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED:
        return "accepted";
      case VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
        return "temporarily rejected";
      case VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED:
        return "denied";
      case VehicleCommandAck::VEHICLE_CMD_RESULT_UNSUPPORTED:
        return "unsupported";
      case VehicleCommandAck::VEHICLE_CMD_RESULT_FAILED:
        return "failed";
      case VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS:
        return "in progress";
      case VehicleCommandAck::VEHICLE_CMD_RESULT_CANCELLED:
        return "cancelled";
      default:
        return "unknown";
    }
  }

  enum class State {
    init,
    position_mode_requested,
    wait_for_stable_position_mode,
    arm_requested,
    armed
  };

  void timer_callback()
  {
  }
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<ManualControlSetpoint>::SharedPtr manual_control_input_publisher_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_position_subscriber_;
  rclcpp::Subscription<LaserScan>::SharedPtr lidar_data_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  State state_;
  uint8_t service_result_;
  bool service_done_;
  int arm_retry_count_;
  rclcpp::Time arm_retry_time_;

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
  void requestVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
  void vehicle_response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
  void lidar_data_callback(const LaserScan::SharedPtr msg);
};

void ManualControlNode::requestVehicleCommand(uint16_t command, float param1, float param2)
{
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

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
	request->request = msg;
  service_done_ = false;
	auto result = vehicle_command_client_->async_send_request(request, std::bind(&ManualControlNode::vehicle_response_callback, this,
                           std::placeholders::_1));
}

void ManualControlNode::vehicle_response_callback(
  rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
  auto status = future.wait_for(1s);
  if (status == std::future_status::ready) {
  auto reply = future.get()->reply;
  service_result_ = reply.result;
  service_done_ = true;
  }
}

void ManualControlNode::armDrone() {
  requestVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

void ManualControlNode::disarmDrone() {
  requestVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

void ManualControlNode::lidar_data_callback(const LaserScan::SharedPtr msg) {
  (void)msg;
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

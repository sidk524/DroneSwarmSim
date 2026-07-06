#include <Eigen/Eigen>
#include <fcntl.h>
#include <linux/input.h>
#include <px4_ros2/common/exception.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sys/ioctl.h>
#include <unistd.h>
#include <X11/Xlib.h>
#include <X11/keysym.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace
{
constexpr const char * kModeName = "Keyboard Flight";
constexpr const char * kNodeName = "keyboard_flight_mode";
constexpr int kKeyCount = KEY_MAX + 1;

bool testBit(int bit, const std::vector<unsigned long> & bits)
{
  constexpr int bits_per_word = static_cast<int>(sizeof(unsigned long) * 8);
  const int word = bit / bits_per_word;
  const int offset = bit % bits_per_word;

  if (word < 0 || word >= static_cast<int>(bits.size())) {
    return false;
  }

  return (bits[word] & (1UL << offset)) != 0;
}

bool hasKeyboardControls(int fd)
{
  std::vector<unsigned long> ev_bits((EV_MAX + sizeof(unsigned long) * 8) /
                                     (sizeof(unsigned long) * 8));
  if (ioctl(fd, EVIOCGBIT(0, ev_bits.size() * sizeof(unsigned long)), ev_bits.data()) < 0) {
    return false;
  }

  if (!testBit(EV_KEY, ev_bits)) {
    return false;
  }

  std::vector<unsigned long> key_bits((KEY_MAX + sizeof(unsigned long) * 8) /
                                      (sizeof(unsigned long) * 8));
  if (ioctl(fd, EVIOCGBIT(EV_KEY, key_bits.size() * sizeof(unsigned long)), key_bits.data()) < 0) {
    return false;
  }

  return testBit(KEY_W, key_bits) && testBit(KEY_A, key_bits) && testBit(KEY_S, key_bits) &&
         testBit(KEY_D, key_bits);
}

std::string deviceName(int fd)
{
  std::array<char, 256> name{};
  if (ioctl(fd, EVIOCGNAME(name.size()), name.data()) < 0) {
    return "unknown";
  }

  return std::string{name.data()};
}
}  // namespace

class KeyboardFlightMode : public px4_ros2::ModeBase
{
public:
  explicit KeyboardFlightMode(rclcpp::Node & node)
  : ModeBase(node, Settings{kModeName}),
    _trajectory_setpoint(std::make_shared<px4_ros2::TrajectorySetpointType>(*this)),
    _attitude(std::make_shared<px4_ros2::OdometryAttitude>(*this))
  {
    _max_xy_speed_m_s = node.declare_parameter<float>("max_xy_speed_m_s", 2.0f);
    _max_z_speed_m_s = node.declare_parameter<float>("max_z_speed_m_s", 1.0f);
    _max_yaw_rate_rad_s =
      px4_ros2::degToRad(node.declare_parameter<float>("max_yaw_rate_deg_s", 60.0f));
    _boost_multiplier = node.declare_parameter<float>("boost_multiplier", 2.0f);
    _input_device = node.declare_parameter<std::string>("input_device", "");

    setSetpointUpdateRate(50.0f);
    openKeyboardDevices();

    _poll_timer = node.create_wall_timer(
      std::chrono::milliseconds(10), [this]() { pollKeyboardDevices(); });

    RCLCPP_INFO(
      node.get_logger(),
      "Keyboard controls: W/S forward/back, A/D left/right, R/F up/down, Q/E yaw, Shift boost");
  }

  ~KeyboardFlightMode() override
  {
    for (const int fd : _keyboard_fds) {
      close(fd);
    }

    if (_x11_display != nullptr) {
      XCloseDisplay(_x11_display);
    }
  }

  void onActivate() override
  {
    _keys.fill(false);
    RCLCPP_INFO(node().get_logger(), "Keyboard Flight mode active");
  }

  void onDeactivate() override
  {
    _keys.fill(false);
    RCLCPP_INFO(node().get_logger(), "Keyboard Flight mode inactive");
  }

  void updateSetpoint(float /*dt_s*/) override
  {
    pollKeyboardDevices();

    const float forward =
      keyValue(KEY_W) - keyValue(KEY_S);  // body forward/back, positive forward
    const float right = keyValue(KEY_D) - keyValue(KEY_A);  // body right/left, positive right
    const float down = keyValue(KEY_F) - keyValue(KEY_R);   // NED down, positive down
    const float yaw_rate = (keyValue(KEY_E) - keyValue(KEY_Q)) * _max_yaw_rate_rad_s;

    Eigen::Vector2f horizontal_body{forward, right};
    if (horizontal_body.norm() > 1.0f) {
      horizontal_body.normalize();
    }

    const float speed_scale =
      (isPressed(KEY_LEFTSHIFT) || isPressed(KEY_RIGHTSHIFT)) ? _boost_multiplier : 1.0f;
    horizontal_body *= _max_xy_speed_m_s * speed_scale;

    const float yaw = _attitude->yaw();
    const float cos_yaw = std::cos(yaw);
    const float sin_yaw = std::sin(yaw);

    const Eigen::Vector3f velocity_ned_m_s{
      cos_yaw * horizontal_body.x() - sin_yaw * horizontal_body.y(),
      sin_yaw * horizontal_body.x() + cos_yaw * horizontal_body.y(),
      std::clamp(down, -1.0f, 1.0f) * _max_z_speed_m_s * speed_scale};

    _trajectory_setpoint->update(velocity_ned_m_s, {}, {}, yaw_rate);
  }

private:
  void openKeyboardDevices()
  {
    openX11Display();

    if (!_input_device.empty()) {
      openKeyboardDevice(_input_device, true);
      return;
    }

    const std::filesystem::path input_dir{"/dev/input"};
    if (!std::filesystem::exists(input_dir)) {
      RCLCPP_WARN(node().get_logger(), "/dev/input does not exist; keyboard input unavailable");
      return;
    }

    for (const auto & entry : std::filesystem::directory_iterator(input_dir)) {
      const auto path = entry.path();
      if (path.filename().string().rfind("event", 0) == 0) {
        openKeyboardDevice(path.string(), false);
      }
    }

    if (_keyboard_fds.empty() && _x11_display == nullptr) {
      RCLCPP_WARN(
        node().get_logger(),
        "No keyboard input backend available. Try input_device:=/dev/input/eventX, add your user "
        "to the input group, or run under an X11/Xwayland session.");
    }
  }

  void openX11Display()
  {
    _x11_display = XOpenDisplay(nullptr);
    if (_x11_display == nullptr) {
      RCLCPP_WARN(node().get_logger(), "Could not open X11 display for keyboard fallback");
      return;
    }

    _x11_w = XKeysymToKeycode(_x11_display, XK_w);
    _x11_a = XKeysymToKeycode(_x11_display, XK_a);
    _x11_s = XKeysymToKeycode(_x11_display, XK_s);
    _x11_d = XKeysymToKeycode(_x11_display, XK_d);
    _x11_q = XKeysymToKeycode(_x11_display, XK_q);
    _x11_e = XKeysymToKeycode(_x11_display, XK_e);
    _x11_r = XKeysymToKeycode(_x11_display, XK_r);
    _x11_f = XKeysymToKeycode(_x11_display, XK_f);
    _x11_shift_l = XKeysymToKeycode(_x11_display, XK_Shift_L);
    _x11_shift_r = XKeysymToKeycode(_x11_display, XK_Shift_R);

    RCLCPP_INFO(node().get_logger(), "Using X11 keyboard fallback on display %s", DisplayString(_x11_display));
  }

  void openKeyboardDevice(const std::string & path, bool warn_on_failure)
  {
    const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
      if (warn_on_failure || errno != EACCES) {
        RCLCPP_WARN(
          node().get_logger(), "Could not open %s: %s", path.c_str(), std::strerror(errno));
      }
      return;
    }

    if (!hasKeyboardControls(fd)) {
      close(fd);
      return;
    }

    _keyboard_fds.push_back(fd);
    RCLCPP_INFO(
      node().get_logger(), "Using keyboard input device %s (%s)", path.c_str(),
      deviceName(fd).c_str());
  }

  void pollKeyboardDevices()
  {
    pollX11Keyboard();

    input_event event{};

    for (const int fd : _keyboard_fds) {
      while (true) {
        const ssize_t bytes_read = read(fd, &event, sizeof(event));
        if (bytes_read == static_cast<ssize_t>(sizeof(event))) {
          if (event.type == EV_KEY && event.code < _keys.size()) {
            _keys[event.code] = event.value != 0;
          }
          continue;
        }

        if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
          break;
        }

        break;
      }
    }
  }

  void pollX11Keyboard()
  {
    if (_x11_display == nullptr) {
      return;
    }

    char keymap[32]{};
    XQueryKeymap(_x11_display, keymap);

    _x11_w_pressed = x11KeyPressed(keymap, _x11_w);
    _x11_a_pressed = x11KeyPressed(keymap, _x11_a);
    _x11_s_pressed = x11KeyPressed(keymap, _x11_s);
    _x11_d_pressed = x11KeyPressed(keymap, _x11_d);
    _x11_q_pressed = x11KeyPressed(keymap, _x11_q);
    _x11_e_pressed = x11KeyPressed(keymap, _x11_e);
    _x11_r_pressed = x11KeyPressed(keymap, _x11_r);
    _x11_f_pressed = x11KeyPressed(keymap, _x11_f);
    _x11_shift_l_pressed = x11KeyPressed(keymap, _x11_shift_l);
    _x11_shift_r_pressed = x11KeyPressed(keymap, _x11_shift_r);
  }

  bool x11KeyPressed(const char keymap[32], KeyCode key_code) const
  {
    if (key_code == 0) {
      return false;
    }

    return (keymap[key_code / 8] & (1 << (key_code % 8))) != 0;
  }

  bool isPressed(int key_code) const
  {
    if (key_code >= 0 && key_code < static_cast<int>(_keys.size()) && _keys[key_code]) {
      return true;
    }

    switch (key_code) {
      case KEY_W:
        return _x11_w_pressed;
      case KEY_A:
        return _x11_a_pressed;
      case KEY_S:
        return _x11_s_pressed;
      case KEY_D:
        return _x11_d_pressed;
      case KEY_Q:
        return _x11_q_pressed;
      case KEY_E:
        return _x11_e_pressed;
      case KEY_R:
        return _x11_r_pressed;
      case KEY_F:
        return _x11_f_pressed;
      case KEY_LEFTSHIFT:
        return _x11_shift_l_pressed;
      case KEY_RIGHTSHIFT:
        return _x11_shift_r_pressed;
      default:
        return false;
    }
  }

  float keyValue(int key_code) const { return isPressed(key_code) ? 1.0f : 0.0f; }

  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  std::shared_ptr<px4_ros2::OdometryAttitude> _attitude;
  rclcpp::TimerBase::SharedPtr _poll_timer;
  std::vector<int> _keyboard_fds;
  std::array<bool, kKeyCount> _keys{};
  Display * _x11_display{nullptr};
  KeyCode _x11_w{0};
  KeyCode _x11_a{0};
  KeyCode _x11_s{0};
  KeyCode _x11_d{0};
  KeyCode _x11_q{0};
  KeyCode _x11_e{0};
  KeyCode _x11_r{0};
  KeyCode _x11_f{0};
  KeyCode _x11_shift_l{0};
  KeyCode _x11_shift_r{0};
  bool _x11_w_pressed{false};
  bool _x11_a_pressed{false};
  bool _x11_s_pressed{false};
  bool _x11_d_pressed{false};
  bool _x11_q_pressed{false};
  bool _x11_e_pressed{false};
  bool _x11_r_pressed{false};
  bool _x11_f_pressed{false};
  bool _x11_shift_l_pressed{false};
  bool _x11_shift_r_pressed{false};
  std::string _input_device;
  float _max_xy_speed_m_s{2.0f};
  float _max_z_speed_m_s{1.0f};
  float _max_yaw_rate_rad_s{px4_ros2::degToRad(60.0f)};
  float _boost_multiplier{2.0f};
};

using KeyboardFlightNode = px4_ros2::NodeWithMode<KeyboardFlightMode>;

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardFlightNode>(kNodeName, true));
    rclcpp::shutdown();
  } catch (const px4_ros2::Exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }

  return 0;
}

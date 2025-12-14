#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node
{
public:
  TestNode() : Node("test_node")
  {
    RCLCPP_INFO(this->get_logger(), "TestNode started");

    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&TestNode::timer_callback, this)
    );
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Tick");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}

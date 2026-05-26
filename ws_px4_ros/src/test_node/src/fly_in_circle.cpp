#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <rclcpp/create_client.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <stdint.h>

#include <string>
#include <vector>


using namespace px4_msgs::msg;

using namespace std;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber(string px4_namespace) : 
        Node("fly_in_circle_node"),
        state{State::flying_up}
    {
      positionSubscription_ = this->create_subscription<VehicleLocalPosition>(
        px4_namespace + "out/vehicle_local_position_v1", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
        trajectoryPublisher = this->create_publisher<TrajectorySetpoint>(px4_namespace + "in/trajectory_setpoint", 10);
    }

  private:  
    enum class State{
        flying_up,
        flying_in_circle
    };

    State state;

    void topic_callback(const VehicleLocalPosition::SharedPtr msg);

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr positionSubscription_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboardPublisher;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectoryPublisher;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicleCommandClient;

};

void MinimalSubscriber::topic_callback(const VehicleLocalPosition::SharedPtr msg) {
    if (state == State::flying_up){
        if (msg->z >= -19.5 && msg->z <= -20.5){
            state = State::flying_in_circle;
        }
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>("/fmu/"));
  rclcpp::shutdown();
  return 0;
}
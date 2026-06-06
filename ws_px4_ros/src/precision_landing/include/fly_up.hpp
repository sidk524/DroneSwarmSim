#include "rclcpp/rclcpp.hpp"
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>



class InitialFlyUpMode : public px4_ros2::ModeBase // [1]
{
public:
  explicit InitialFlyUpMode(rclcpp::Node & node) : ModeBase(node, Settings{"Initial Fly Up Mode"}) // [2]
  {
    // [3]
    trajectorySetpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    initialCoords = {};
    initialCoords = initialCoords.withPositionX(-20.0).withPositionY(0.0).withPositionZ(-20.0);
  
  }

  void onActivate() override
  {
    fly_up();
  }

  void onDeactivate() override
  {
    // Called when our mode gets deactivated
  }

private:
  void fly_up();

  std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectorySetpoint;
  px4_ros2::TrajectorySetpoint initialCoords;


};

void InitialFlyUpMode::fly_up() {
  trajectorySetpoint->update(initialCoords);
}



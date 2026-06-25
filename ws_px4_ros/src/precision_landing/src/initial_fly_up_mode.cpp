
#include <fly_up.hpp>
#include "rclcpp/rclcpp.hpp"
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>


InitialFlyUpMode::InitialFlyUpMode(rclcpp::Node & node) : ModeBase(node, Settings{"Initial Fly Up Mode"}),
  _node(node)
{
    
    trajectorySetpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    localPosition = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

  }

void InitialFlyUpMode::onActivate() {

    initialCoords = {};
    initialCoords = initialCoords.withPositionX(x).withPositionY(y).withPositionZ(z);
    timer = _node.create_wall_timer(1000ms, std::bind(&InitialFlyUpMode::position_poll, this));
    fly_up();
}

void InitialFlyUpMode::onDeactivate(){
    timer->cancel();
    localPosition.reset();
    trajectorySetpoint.reset();
}

void InitialFlyUpMode::fly_up() {
  trajectorySetpoint->update(initialCoords);
}

void InitialFlyUpMode::position_poll(){
  currLocalCoords = localPosition->positionNed();
  

  if ((currLocalCoords.x() > x - 0.5 && currLocalCoords.x() < x + 0.5) &&  
      (currLocalCoords.y() > y - 0.5 && currLocalCoords.y() < y + 0.5) &&
      (currLocalCoords.z() > z - 0.5 && currLocalCoords.z() < z + 0.5)){
          // timer->cancel();
          // localPosition.reset();
          // trajectorySetpoint.reset();
          completed(px4_ros2::Result::Success);
          
  }
}

// int main(int argc, char* argv[]){

//     using initialFlyUpNode = px4_ros2::NodeWithMode<InitialFlyUpMode>;
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<initialFlyUpNode>("node_with_mode", true));
//     rclcpp::shutdown();

// }


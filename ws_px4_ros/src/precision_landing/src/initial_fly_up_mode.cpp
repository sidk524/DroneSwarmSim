
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
    initialCoords = initialCoords.withPositionX(2.0).withPositionY(3.0).withPositionZ(-5.0);
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
  
  RCLCPP_DEBUG(_node.get_logger(), "POSITION POLL x: %f, y:%f, z:%f",
      currLocalCoords.x(), currLocalCoords.y(), currLocalCoords.z()
    );

  if ((currLocalCoords.x() > 1.5 && currLocalCoords.x() < 2.5) &&  
      (currLocalCoords.y() > 2.5 && currLocalCoords.y() < 3.5) &&
      (currLocalCoords.z() < -4.5 && currLocalCoords.z() > -5.5)){

    RCLCPP_DEBUG(_node.get_logger(), "initial fly mode should finish here"
    );
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


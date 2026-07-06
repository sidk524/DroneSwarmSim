#include "geometry_msgs/msg/vector3.hpp"
#include <memory>
#include <move_above_marker.hpp>
#include <px4_ros2/components/mode.hpp>
#include <rclcpp/duration.hpp>
#include <tf2/convert.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

MoveAboveMarkerMode::MoveAboveMarkerMode(rclcpp::Node & node) : ModeBase(node, Settings("Move above marker mode")),
  _node(node)
{
    trajectorySetpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    localPosition = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
}

void MoveAboveMarkerMode::onActivate() {
    arucoMarkerSubscriber = _node.create_subscription<geometry_msgs::msg::Vector3>( "/aruco_marker_position", 10, 
        std::bind(&MoveAboveMarkerMode::arucoCallback, this, std::placeholders::_1));
    timer = _node.create_wall_timer(500ms, std::bind(&MoveAboveMarkerMode::checkCompletion, this));
}

void MoveAboveMarkerMode::checkCompletion() {
    Eigen::Vector3f currLocalCoords = localPosition->positionNed();
     if ((currLocalCoords.x() > lastArucoPosition[0] - 0.5 && currLocalCoords.x() < lastArucoPosition[0] + 0.5) &&  
      (currLocalCoords.y() > lastArucoPosition[1] - 0.5 && currLocalCoords.y() < lastArucoPosition[1] + 0.5)){
        
        completed(px4_ros2::Result::Success);
    }
}

void MoveAboveMarkerMode::arucoCallback(geometry_msgs::msg::Vector3 msg) {
    arucoCoords = {};
    lastArucoPosition = {msg.x, msg.y, msg.z};
    arucoCoords = arucoCoords.withPositionX(msg.x).withPositionY(msg.y).withPositionZ(localPosition->positionNed().z());
    trajectorySetpoint->update(arucoCoords);
}
 
void MoveAboveMarkerMode::onDeactivate() {
    arucoMarkerSubscriber.reset();
    timer->cancel();
}

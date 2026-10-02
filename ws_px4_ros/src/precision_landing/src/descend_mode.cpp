#include "geometry_msgs/msg/vector3.hpp"
#include <descend_mode.hpp>
#include <memory>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include <px4_ros2/vehicle_state/land_detected.hpp>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

DescendMode::DescendMode(rclcpp::Node & node) : px4_ros2::ModeBase(node, Settings{"Descend Mode"}),
    _node(node)
    {
        trajectorySetpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);



        landingDetected = std::make_shared<px4_ros2::LandDetected>(*this);

        localPosition = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);


        descendPosition = {};
        

    }

void DescendMode::arucoMarkerCallback(geometry_msgs::msg::Vector3 msg){
    const Eigen::Vector3f targetNed = px4_ros2::positionEnuToNed(Eigen::Vector3f(msg.x, msg.y, msg.z));
    targetZ = -msg.z;
    descendPosition = {};
    descendPosition.withPositionX(targetNed.x()).withPositionY(targetNed.y()).withVelocityZ(0.5);
    trajectorySetpoint->update(descendPosition);
}

void DescendMode::check_landing() {
    RCLCPP_INFO(_node.get_logger(), "%s", landingDetected->landed() ? "true" : "false");
    if (std::abs(targetZ - localPosition->positionNed()[2]) < 1.0){
        arucoMarkerPositionSubscriber.reset();
        timer->cancel();
        completed(px4_ros2::Result::Success);
    }
}

void DescendMode::onActivate() {
    arucoMarkerPositionSubscriber = _node.create_subscription<geometry_msgs::msg::Vector3>("/aruco_marker_position", qosProfile, std::bind(&DescendMode::arucoMarkerCallback, this, std::placeholders::_1));

    timer = _node.create_wall_timer(1s, std::bind(&DescendMode::check_landing, this));

}

void DescendMode::onDeactivate() {
    arucoMarkerPositionSubscriber.reset();
    timer->cancel();
}
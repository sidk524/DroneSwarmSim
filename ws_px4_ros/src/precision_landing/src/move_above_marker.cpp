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
    buffer = std::make_unique<tf2_ros::Buffer>(_node.get_clock() );
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    trajectorySetpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

}

void MoveAboveMarkerMode::onActivate() {
    timer = _node.create_wall_timer(1s, std::bind(&MoveAboveMarkerMode::log_aruco_position, this));

}

void MoveAboveMarkerMode::onDeactivate() {

}

void MoveAboveMarkerMode::log_aruco_position(){
    geometry_msgs::msg::PointStamped aruco_point;

    aruco_point.header.stamp = _node.get_clock()->now();
    aruco_point.header.frame_id = "aruco_marker_frame";
    aruco_point.point.x = 0.0;
    aruco_point.point.y = 0.0;
    aruco_point.point.z = 0.0;

    geometry_msgs::msg::PointStamped odom_aruco_point;
    
    auto transformStamped = buffer->lookupTransform(
        "odom", "aruco_marker_frame", tf2::TimePointZero, tf2::Duration(1)
    );

    tf2::doTransform(aruco_point, odom_aruco_point, transformStamped);

    RCLCPP_DEBUG(_node.get_logger(), "x: %f y: %f z: %f",
        odom_aruco_point.point.x, odom_aruco_point.point.y, odom_aruco_point.point.z 
    );
}

#include <array>
#include <cmath>
#include <locate_marker.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "my_msgs/msg/tvec_rvec.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <rcl/publisher.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <rcutils/logging.h>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

LocateArucoMarkerMode::LocateArucoMarkerMode(rclcpp::Node& node) : 
    ModeBase(node, Settings{"Locate Aruco Marker Mode"}),
    _node(node)
    {   
        trajectorySetpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        localPosition = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    }

void LocateArucoMarkerMode::onActivate(){

    tvecRvecSubscriber = _node.create_subscription<geometry_msgs::msg::Vector3>("/aruco_marker_position", qosProfile,
        std::bind(&LocateArucoMarkerMode::tvecRvecCallback, this, std::placeholders::_1)
    );  

    centreOfCircle = localPosition->positionNed();
    
    currentYawAngle = 0.0; 
    
    dt = 0.01; 

    radiusTimer = _node.create_wall_timer(10ms, std::bind(&LocateArucoMarkerMode::emitCircleWaypoints, this));
    RCLCPP_DEBUG(_node.get_logger(), "locate aruco marker mode activated");
}

void LocateArucoMarkerMode::emitCircleWaypoints(){
    radius += 0.001; 

    if (radius < 0.1) radius = 0.1;

    float yaw_rate = circleFlyVelocity / radius;

    

    currentYawAngle += yaw_rate * dt;
    currentYawAngle = std::atan2(std::sin(currentYawAngle), std::cos(currentYawAngle));

    float velocity_x = circleFlyVelocity * std::cos(currentYawAngle);
    float velocity_y = circleFlyVelocity * std::sin(currentYawAngle);

    float pos_x = centreOfCircle.x() + radius * std::sin(currentYawAngle);
    float pos_y = centreOfCircle.y() - radius * std::cos(currentYawAngle);
    float pos_z = centreOfCircle.z(); 

    px4_ros2::TrajectorySetpoint arucoCoords = {};
    
    arucoCoords = arucoCoords.withPosition({pos_x, pos_y, pos_z})
                             .withVelocityX(velocity_x)
                             .withVelocityY(velocity_y)
                             .withVelocityZ(0.0f)
                             .withYaw(currentYawAngle)
                             .withYawRate(yaw_rate);

    trajectorySetpoint->update(arucoCoords);
}   

void LocateArucoMarkerMode::tvecRvecCallback(geometry_msgs::msg::Vector3 msg){
    RCLCPP_DEBUG(_node.get_logger(), "callback received");
    completed(px4_ros2::Result::Success);
}

void LocateArucoMarkerMode::onDeactivate(){
    tvecRvecSubscriber.reset();
    radiusTimer->cancel();
}

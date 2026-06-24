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

LocateArucoMarkerMode::LocateArucoMarkerMode(rclcpp::Node& node) : 
    ModeBase(node, Settings{"Locate Aruco Marker Mode"}),
    _node(node)
    {   
        trajectorySetpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        localPosition = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    }

void LocateArucoMarkerMode::onActivate(){
    RCLCPP_DEBUG(_node.get_logger(), "locate aruco marker mode activated");

    tvecRvecSubscriber = _node.create_subscription<my_msgs::msg::TvecRvec>("/tvec_rvec",10,
        std::bind(&LocateArucoMarkerMode::tvecRvecCallback, this, std::placeholders::_1)
    );

    tfStaticTransformPublisher = std::make_shared<tf2_ros::StaticTransformBroadcaster>(&_node);
}

void LocateArucoMarkerMode::tvecRvecCallback(my_msgs::msg::TvecRvec msg){
    completed(px4_ros2::Result::Success);
}

void LocateArucoMarkerMode::onDeactivate(){
    tvecRvecSubscriber.reset();
    
}



// int main(int argc, char* argv[]){
//     using locateMarkerNode = px4_ros2::NodeWithMode<LocateArucoMarkerMode>;
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<locateMarkerNode>("node_with_mode", true));
//     rclcpp::shutdown();
// }



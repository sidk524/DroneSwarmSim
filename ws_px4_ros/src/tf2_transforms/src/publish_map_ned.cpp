#include <functional>
#include <memory>
#include <publish_map_ned.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

#include <tf2_ros/static_transform_broadcaster.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "rclcpp/rclcpp.hpp"


MapNedPublisher::MapNedPublisher() : rclcpp::Node("MapNedPublisher"){
    globalPositionSubscriber = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position", qosProfile, std::bind(&MapNedPublisher::globalPositionCallback, this, std::placeholders::_1));

        tfStaticTransformPublisher = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

void MapNedPublisher::globalPositionCallback(px4_msgs::msg::VehicleGlobalPosition msg){
    RCLCPP_DEBUG(this->get_logger(), "sending transform");
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "map_ned";

    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    
    transform.transform.rotation.w = 0.5;
    transform.transform.rotation.x = 0.5;
    transform.transform.rotation.y = 0.5;
    transform.transform.rotation.z = 0.5;

    tfStaticTransformPublisher->sendTransform(transform);
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapNedPublisher>());
    rclcpp::shutdown();
}
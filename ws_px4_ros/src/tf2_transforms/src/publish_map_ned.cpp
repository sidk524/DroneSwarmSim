#include <functional>
#include <publish_map_ned.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "rclcpp/rclcpp.hpp"


MapNedPublisher::MapNedPublisher() : rclcpp::Node("MapNedPublisher"){
    globalPositionSubscriber = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position", 10, std::bind(&MapNedPublisher::globalPositionCallback, this, std::placeholders::_1));
    
        tfStaticTransformPublisher = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

void MapNedPublisher::globalPositionCallback(px4_msgs::msg::VehicleGlobalPosition msg){
    if (!transformPublished) {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";

        transform.transform.translation.x = 0;
        transform.transform.translation.y = 0;
        transform.transform.translation.z = 0;
    }
}
#include <publish_camera_optical.hpp>
#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

#include <tf2_ros/static_transform_broadcaster.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/create_client.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rmw/types.h>

using namespace std::chrono_literals;



CameraOpticalPublisher::CameraOpticalPublisher() : rclcpp::Node("camera_optical_publisher")
{   
    tfStaticTransformPublisher = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    q1.setRPY(0,1.57079632679, 0);
    q2.setRPY(0,0,1.57079632679);
    qtotal = q1 * q2;
    qtotal.normalize();
    
    t.header.frame_id = "camera_link";
    t.header.stamp = this->get_clock()->now();

    t.child_frame_id = "camera_optical_frame";
    t.transform.translation.x = 0;
    t.transform.translation.y = 0;
    t.transform.translation.z = 0;

    t.transform.rotation.w = qtotal.w();
    t.transform.rotation.x = qtotal.x();
    t.transform.rotation.y = qtotal.y();
    t.transform.rotation.z = qtotal.z();
    wallTimer = this->create_wall_timer(100ms, std::bind(&CameraOpticalPublisher::publishTransform, this));
}

void CameraOpticalPublisher::publishTransform(){
    RCLCPP_DEBUG(this->get_logger(), "sending transform");
    tfStaticTransformPublisher->sendTransform(t);

}

int main (int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraOpticalPublisher>());
    rclcpp::shutdown();
}


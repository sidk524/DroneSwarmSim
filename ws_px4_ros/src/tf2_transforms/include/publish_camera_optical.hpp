#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <rclcpp/timer.hpp>
#include <rmw/types.h>
#include <rmw/qos_profiles.h>
#include <rclcpp/create_client.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rmw/types.h>



class CameraOpticalPublisher : public rclcpp::Node {

    public:
        explicit CameraOpticalPublisher();
    
    private:
        tf2::Quaternion q1;
        tf2::Quaternion q2;
        tf2::Quaternion qtotal;

        geometry_msgs::msg::TransformStamped t;

        rclcpp::TimerBase::SharedPtr wallTimer;

        void publishTransform();
        
        const rclcpp::QoS qosProfile = rclcpp::QoS(10).reliability_best_available();
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfStaticTransformPublisher;

};
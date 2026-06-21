#include <memory>
#include <rclcpp/subscription.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include <px4_msgs/msg/vehicle_global_position.hpp>



class MapNedPublisher : rclcpp::Node 
{
    
    public:
        explicit MapNedPublisher();

    private:
        rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr globalPositionSubscriber;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfStaticTransformPublisher;
        void globalPositionCallback(px4_msgs::msg::VehicleGlobalPosition msg);
        bool transformPublished = false;
};


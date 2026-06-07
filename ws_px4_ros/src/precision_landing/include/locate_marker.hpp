#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <memory>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>


#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>


class LocateArucoMarkerMode : public px4_ros2::ModeBase
{
public:
    explicit LocateArucoMarkerMode(rclcpp::Node& node) : 
    ModeBase(node, Settings{"Locate Aruco Marker Mode"}),
    _node(node)
    {
        imageSubscriber = _node.create_subscription<sensor_msgs::msg::Image>("/fmu/out/camera_image", 10,
        std::bind(&LocateArucoMarkerMode::image_callback, this, std::placeholders::_1));
    }
    void onActivate() override;

    void onDeactivate() override;

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber;
    void image_callback(sensor_msgs::msg::Image image_msg);

    rclcpp::Node& _node;
    
};
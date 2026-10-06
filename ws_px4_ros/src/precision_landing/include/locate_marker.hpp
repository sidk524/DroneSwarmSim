#include "my_msgs/msg/tvec_rvec.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <px4_ros2/components/mode.hpp>

#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <px4_ros2/components/mode_executor.hpp>

#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <px4_ros2/utils/vehicle_command_sender.hpp>

#include <px4_ros2/odometry/local_position.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>


class LocateArucoMarkerMode : public px4_ros2::ModeBase
{
public:     
    explicit LocateArucoMarkerMode(rclcpp::Node& node);
    void tvecRvecCallback(geometry_msgs::msg::Vector3 msg);
    void onActivate() override;
    void onDeactivate() override;

    void emitCircleWaypoints();

    void updateRadii();


    rclcpp::Node& _node;

private:

    const rclcpp::QoS qosProfile = rclcpp::QoS(10).reliability_best_available().durability_best_available();

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr tvecRvecSubscriber;
    
    std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectorySetpoint;


    std::shared_ptr<px4_ros2::OdometryLocalPosition> localPosition;

    Eigen::Vector3f centreOfCircle;

    rclcpp::TimerBase::SharedPtr radiusTimer;


    float radius;
    
    float dt;
    float currentYawAngle;


    float circleFlyVelocity = 1.0;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfStaticTransformPublisher;

    
};




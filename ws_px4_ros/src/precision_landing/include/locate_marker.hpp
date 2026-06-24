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

#include <px4_ros2/odometry/local_position.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <vector>

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>


class LocateArucoMarkerMode : public px4_ros2::ModeBase
{
public:     
    explicit LocateArucoMarkerMode(rclcpp::Node& node);
    void tvecRvecCallback(my_msgs::msg::TvecRvec msg);
    void onActivate() override;
    void onDeactivate() override;


    rclcpp::Node& _node;

private:

    rclcpp::Subscription<my_msgs::msg::TvecRvec>::SharedPtr tvecRvecSubscriber;


    std::shared_ptr<px4_ros2::OdometryLocalPosition> localPosition;

    cv::Mat distortionCoefficients;
    cv::Mat cameraMatrix;

    const float arucoMarkerLength = 0.5;
    std::vector<cv::Vec3d> objPoints = {
        {-0.25,0.25, 0},
        {0.25, 0.25, 0},
        {0.25, -0.25, 0},
       { -0.25, -0.25, 0}
    };

    cv::Vec3d tvec;
    cv::Vec3d rvec;


    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::Dictionary dictionary;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectorySetpoint;
    
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::ArucoDetector detector(cv::aruco::Dictionary, cv::aruco::DetectorParameters);

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfStaticTransformPublisher;

    std::vector<int> markerIds;
    
};




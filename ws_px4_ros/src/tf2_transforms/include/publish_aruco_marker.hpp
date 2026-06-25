#include <memory>
#include <opencv4/opencv2/core/matx.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>

#include "geometry_msgs/msg/vector3.hpp"
#include "my_msgs/msg/tvec_rvec.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
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
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/objdetect/aruco_detector.hpp>
#include <my_msgs/msg/tvec_rvec.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/camera_info.hpp"

#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

class PublishArucoMarkerFrame : public rclcpp::Node {
    public: 
        explicit PublishArucoMarkerFrame();
        void publish_aruco_marker_frame(cv::Vec3d tvec, cv::Vec3d rvec);
        void tvec_rvec_callback(my_msgs::msg::TvecRvec msg);

        void image_callback(sensor_msgs::msg::Image::SharedPtr image_msg);
        void image_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr image_info_msg);
    
    private:
        const rclcpp::QoS qosProfile = rclcpp::QoS(10).reliability_best_available().durability_best_available();


        std::unique_ptr<tf2_ros::Buffer> buffer;
        std::shared_ptr<tf2_ros::TransformListener> listener;

        rclcpp::Publisher<my_msgs::msg::TvecRvec>::SharedPtr tvecRvecPublisher;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr arucoMarkerPosition;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr imageInfoSubscriber;


        std::shared_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;

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
        
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::aruco::ArucoDetector detector(cv::aruco::Dictionary, cv::aruco::DetectorParameters);
        std::vector<int> markerIds;


        
};
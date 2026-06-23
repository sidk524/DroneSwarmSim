#include <array>
#include <cmath>
#include <locate_marker.hpp>
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <opencv2/core/hal/interface.h>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <rcutils/logging.h>
#include <vector>

LocateArucoMarkerMode::LocateArucoMarkerMode(rclcpp::Node& node) : 
    ModeBase(node, Settings{"Locate Aruco Marker Mode"}),
    _node(node)
    {   
        trajectorySetpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        localPosition = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    }

void LocateArucoMarkerMode::onActivate(){
    RCLCPP_DEBUG(_node.get_logger(), "locate aruco marker mode activated");
    imageSubscriber = _node.create_subscription<sensor_msgs::msg::Image>("/fmu/out/camera_image", 10,
    std::bind(&LocateArucoMarkerMode::image_callback, this, std::placeholders::_1));
    imageInfoSubscriber = _node.create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", 
        10, std::bind(&LocateArucoMarkerMode::image_info_callback, this, std::placeholders::_1));
    
}

void LocateArucoMarkerMode::onDeactivate(){
    imageSubscriber.reset();
    imageInfoSubscriber.reset();
}

void LocateArucoMarkerMode::image_callback(sensor_msgs::msg::Image::SharedPtr image_msg) {
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    detector.detectMarkers(cv_ptr->image, markerCorners, markerIds, rejectedCandidates);
    if (markerIds.size() > 0){
        cv::solvePnP(objPoints, markerCorners[0],    
            cameraMatrix, distortionCoefficients, rvec, tvec);
        RCLCPP_DEBUG(_node.get_logger(), "[%f %f %f]",
            tvec[0], tvec[1], tvec[2]); 
            poseAboveMarker = localPosition->positionNed();
            // poseAboveMarker =
            
        completed(px4_ros2::Result::Success);
    }
}


void LocateArucoMarkerMode::image_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr image_info_msg){
    cameraMatrix = cv::Mat(3, 3, CV_64F, image_info_msg->k.data()).clone();

    distortionCoefficients = cv::Mat(1, image_info_msg->d.size(), CV_64F, 
    image_info_msg->d.data()).clone();

    if (!cameraMatrix.empty() && !distortionCoefficients.empty()){
        imageInfoSubscriber.reset();
    }
}   


// int main(int argc, char* argv[]){
//     using locateMarkerNode = px4_ros2::NodeWithMode<LocateArucoMarkerMode>;
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<locateMarkerNode>("node_with_mode", true));
//     rclcpp::shutdown();
// }



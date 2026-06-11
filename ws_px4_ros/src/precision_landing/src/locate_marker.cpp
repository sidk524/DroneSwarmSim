#include <array>
#include <locate_marker.hpp>
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/hal/interface.h>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

void LocateArucoMarkerMode::image_callback(sensor_msgs::msg::Image::SharedPtr image_msg) {
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    detector.detectMarkers(cv_ptr->image, markerCorners, markerIds, rejectedCandidates);
    if (markerIds.size() > 0){
        RCLCPP_DEBUG(_node.get_logger(), "marker ");

        cv::solvePnP(objPoints, markerCorners[0],    
            cameraMatrix, distortionCoefficients, rvec, tvec);
        RCLCPP_DEBUG(_node.get_logger(), "[%f %f %f]",
            tvec[0], tvec[1], tvec[2]); 
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



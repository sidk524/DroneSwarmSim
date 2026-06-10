#include <locate_marker.hpp>
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>


void LocateArucoMarkerMode::image_callback(sensor_msgs::msg::Image::SharedPtr image_msg) {
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    detector.detectMarkers(cv_ptr->image, markerCorners, markerIds, rejectedCandidates);
    if (markerIds.size() > 0){
        RCLCPP_DEBUG(_node.get_logger(), "marker detected");
    }
}

// int main(int argc, char* argv[]){
//     using locateMarkerNode = px4_ros2::NodeWithMode<LocateArucoMarkerMode>;
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<locateMarkerNode>("node_with_mode", true));
//     rclcpp::shutdown();
// }



#include "my_msgs/msg/tvec_rvec.hpp"
#include <memory>
#include <opencv2/core/matx.hpp>
#include <publish_aruco_marker.hpp>
#include <rcl/publisher.h>
#include <tf2_ros/transform_broadcaster.hpp>


PublishArucoMarkerFrame::PublishArucoMarkerFrame() : rclcpp::Node("publish_aruco_frame_node"){
    transformBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tvecRvecPublisher = this->create_publisher<my_msgs::msg::TvecRvec>("/tvec_rvec", 10);
    imageSubscriber = this->create_subscription<sensor_msgs::msg::Image>("/fmu/out/camera_image", 10,
    std::bind(&PublishArucoMarkerFrame::image_callback, this, std::placeholders::_1));


    imageInfoSubscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", 
        10, std::bind(&PublishArucoMarkerFrame::image_info_callback, this, std::placeholders::_1));
    

}

void PublishArucoMarkerFrame::publish_aruco_marker_frame(cv::Vec3d tvec, cv::Vec3d rvec){
    float rvecAngle;
    cv::Vec3d rvecUnitVector;

    geometry_msgs::msg::TransformStamped arucoTransform;

    rvecAngle = std::sqrt(std::pow(rvec[0],2) + std::pow(rvec[1],2 ) + std::pow(rvec[2],2 ));
    rvecUnitVector = rvec / rvecAngle;

    arucoTransform.header.frame_id = "camera_optical_frame";
    arucoTransform.header.stamp = this->get_clock()->now();

    arucoTransform.child_frame_id = "aruco_marker_frame";

    arucoTransform.transform.translation.x = tvec[0];
    arucoTransform.transform.translation.y = tvec[1];
    arucoTransform.transform.translation.z = tvec[2];

    arucoTransform.transform.rotation.w = std::cos(rvecAngle/2);
    arucoTransform.transform.rotation.x = std::sin(rvecAngle/2) * rvecUnitVector[0];
    arucoTransform.transform.rotation.y = std::sin(rvecAngle/2) * rvecUnitVector[1];
    arucoTransform.transform.rotation.z = std::sin(rvecAngle/2) * rvecUnitVector[2];

    transformBroadcaster->sendTransform(arucoTransform);
}


void PublishArucoMarkerFrame::image_callback(sensor_msgs::msg::Image::SharedPtr image_msg) {
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);


    my_msgs::msg::TvecRvec msg;


    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    detector.detectMarkers(cv_ptr->image, markerCorners, markerIds, rejectedCandidates);
    if (markerIds.size() > 0){
        cv::solvePnP(objPoints, markerCorners[0],    
            cameraMatrix, distortionCoefficients, rvec, tvec);

                      
        msg.tvec.x = tvec[0];
        msg.tvec.y = tvec[1];
        msg.tvec.z = tvec[2];

        msg.rvec.x = rvec[0];
        msg.rvec.y = rvec[1];
        msg.rvec.z = rvec[2];
        
        tvecRvecPublisher->publish(msg);
            
        publish_aruco_marker_frame(tvec, rvec);

    }
}


void PublishArucoMarkerFrame::image_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr image_info_msg){
    cameraMatrix = cv::Mat(3, 3, CV_64F, image_info_msg->k.data()).clone();

    distortionCoefficients = cv::Mat(1, image_info_msg->d.size(), CV_64F, 
    image_info_msg->d.data()).clone();

    if (!cameraMatrix.empty() && !distortionCoefficients.empty()){
        imageInfoSubscriber.reset();
    }
}   


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublishArucoMarkerFrame>());
    rclcpp::shutdown();

}
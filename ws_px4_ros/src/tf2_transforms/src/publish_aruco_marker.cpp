#include "geometry_msgs/msg/vector3.hpp"
#include "my_msgs/msg/tvec_rvec.hpp"
#include <memory>
#include <opencv2/core/matx.hpp>
#include <publish_aruco_marker.hpp>
#include <rcl/publisher.h>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.hpp>


PublishArucoMarkerFrame::PublishArucoMarkerFrame() : rclcpp::Node("publish_aruco_frame_node"){
    buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    listener = std::make_unique<tf2_ros::TransformListener>(*buffer);
    transformBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    tvecRvecPublisher = this->create_publisher<my_msgs::msg::TvecRvec>("/tvec_rvec", qosProfile);
    imageSubscriber = this->create_subscription<sensor_msgs::msg::Image>("/fmu/out/camera_image", 10,
    std::bind(&PublishArucoMarkerFrame::image_callback, this, std::placeholders::_1));

    imageInfoSubscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", 
        10, std::bind(&PublishArucoMarkerFrame::image_info_callback, this, std::placeholders::_1));

    arucoMarkerPosition = this-> create_publisher<geometry_msgs::msg::Vector3>("/aruco_marker_position", 10);

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
    
    geometry_msgs::msg::PointStamped aruco_point;
    geometry_msgs::msg::PointStamped odom_aruco_point;
    geometry_msgs::msg::Vector3 odom_aruco_msg;


    aruco_point.header.stamp = this->get_clock()->now();
    aruco_point.header.frame_id = "aruco_marker_frame";
    aruco_point.point.x = 0.0;
    aruco_point.point.y = 0.0;
    aruco_point.point.z = 0.0;

    try {
        auto transformStamped = buffer->lookupTransform(
            "odom", "aruco_marker_frame", tf2::TimePointZero, tf2::Duration(1)
        );
        tf2::doTransform(aruco_point, odom_aruco_point, transformStamped);
        odom_aruco_msg.x = odom_aruco_point.point.x;
        odom_aruco_msg.y = odom_aruco_point.point.y;

        odom_aruco_msg.z = odom_aruco_point.point.z;

        RCLCPP_DEBUG(this->get_logger(), "Publishing aruco marker");

        arucoMarkerPosition->publish(odom_aruco_msg);
    } catch (tf2::TransformException e){

    }




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
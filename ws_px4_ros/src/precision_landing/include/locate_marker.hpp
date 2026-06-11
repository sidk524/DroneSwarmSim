#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <px4_ros2/components/mode.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <px4_ros2/components/mode_executor.hpp>

#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <vector>

class LocateArucoMarkerMode : public px4_ros2::ModeBase
{
public:     
    explicit LocateArucoMarkerMode(rclcpp::Node& node) : 
    ModeBase(node, Settings{"Locate Aruco Marker Mode"}),
    _node(node)
    {
        trajectorySetpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        imageSubscriber = _node.create_subscription<sensor_msgs::msg::Image>("/fmu/out/camera_image", 10,
        std::bind(&LocateArucoMarkerMode::image_callback, this, std::placeholders::_1));
        imageInfoSubscriber = _node.create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", 
            10, std::bind(&LocateArucoMarkerMode::image_info_callback, this, std::placeholders::_1));
    }
    void image_callback(sensor_msgs::msg::Image::SharedPtr image_msg);
    void image_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr image_info_msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr imageInfoSubscriber;

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

    std::vector<int> markerIds;
    rclcpp::Node& _node;
    
};




// # This message contains an uncompressed image
// # (0, 0) is at top-left corner of image

// std_msgs/Header header # Header timestamp should be acquisition time of image
//         builtin_interfaces/Time stamp
//                 int32 sec
//                 uint32 nanosec
//         string frame_id
//                              # Header frame_id should be optical frame of camera
//                              # origin of frame should be optical center of cameara
//                              # +x should point to the right in the image
//                              # +y should point down in the image
//                              # +z should point into to plane of the image
//                              # If the frame_id here and the frame_id of the CameraInfo
//                              # message associated with the image conflict
//                              # the behavior is undefined

// uint32 height                # image height, that is, number of rows
// uint32 width                 # image width, that is, number of columns

// string encoding       # Encoding of pixels -- channel meaning, ordering, size
//                       # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

// uint8 is_bigendian    # is this data bigendian?
// uint32 step           # Full row length in bytes
// uint8[] data          # actual matrix data, size is (step * rows)


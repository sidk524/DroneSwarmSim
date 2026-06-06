#include "rclcpp/rclcpp.hpp"
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>


class LocateArucoMarkerMode : public px4_ros2::ModeBase
{
public:
    explicit LocateArucoMarkerMode(rclcpp::Node & node) : ModeBase(node, Settings{"Locate Aruco Marker Mode"})
    {

    }
    void onActivate() override;

    void onDeactivate() override;

private:
    void locateMarker();
    
};
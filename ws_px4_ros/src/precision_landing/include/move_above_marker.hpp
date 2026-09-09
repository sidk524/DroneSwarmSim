#include "geometry_msgs/msg/vector3.hpp"
#include "my_msgs/msg/tvec_rvec.hpp"
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

#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <px4_ros2/components/mode_executor.hpp>

#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <px4_ros2/odometry/local_position.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"



#include <geometry_msgs/msg/point_stamped.hpp>

class MoveAboveMarkerMode : public px4_ros2::ModeBase {
    public:
        explicit MoveAboveMarkerMode(rclcpp::Node& node);
        rclcpp::Node &_node;
        void onActivate() override;
        void onDeactivate() override;
        void arucoCallback(geometry_msgs::msg::Vector3 msg);
        void checkCompletion();

    private:

        std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectorySetpoint;
        std::shared_ptr<px4_ros2::OdometryLocalPosition> localPosition;
        px4_ros2::TrajectorySetpoint arucoCoords;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr arucoMarkerSubscriber;

        cv::Vec3d lastArucoPosition;

};
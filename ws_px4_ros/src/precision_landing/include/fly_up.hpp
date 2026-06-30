#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <rclcpp/timer.hpp>



class InitialFlyUpMode : public px4_ros2::ModeBase 
{
  public:
    explicit InitialFlyUpMode(rclcpp::Node & node);
    void onActivate() override;
    void onDeactivate() override;
    rclcpp::Node& _node;
    float x = -2.0;
    float y = 3.0;
    float z = -10.0;

  private:
    void fly_up();
    void position_poll();
    std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectorySetpoint;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> localPosition;
    px4_ros2::TrajectorySetpoint initialCoords;
    rclcpp::TimerBase::SharedPtr timer;
    Eigen::Vector3f currLocalCoords;
};


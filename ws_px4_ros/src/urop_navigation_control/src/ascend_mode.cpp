#include <ascend_mode.hpp>
#include <cmath>
using namespace std::chrono_literals;


AscendMode::AscendMode(rclcpp::Node & node) : px4_ros2::ModeBase(node, Settings("Ascend Mode")),
_node(node){
    trajectorySetpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    odomSubscriber = node.create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qosProfile, 
        std::bind(&AscendMode::checkAscended, this, std::placeholders::_1));
}

void AscendMode::onActivate() {
    wall_timer = _node.create_wall_timer(1000ms, std::bind(&AscendMode::ascend, this));


}

void AscendMode::checkAscended(px4_msgs::msg::VehicleOdometry msg) {

    if (std::hypot(msg.position[0], msg.position[1], msg.position[2] + 3) < 1.0){
                completed(px4_ros2::Result::Success);

    }
}

void AscendMode::ascend(){
    Eigen::Vector3f pos = {0.0, 0.0, -3.0};
    trajectorySetpoint->updatePosition(pos);
}

void AscendMode::onDeactivate() {
    odomSubscriber.reset();
    wall_timer->cancel();
}


#include <fly_up.hpp>
#include "rclcpp/rclcpp.hpp"
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>



int main(int argc, char* argv[]){

    using initialFlyUpNode = px4_ros2::NodeWithMode<InitialFlyUpMode>;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<initialFlyUpNode>("node_with_mode", true));
    rclcpp::shutdown();

}


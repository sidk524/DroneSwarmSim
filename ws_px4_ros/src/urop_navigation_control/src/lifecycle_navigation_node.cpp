
#include <memory>
#include <px4_ros2/components/mode.hpp>

#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <px4_ros2/components/mode_executor.hpp>

#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <px4_ros2/odometry/local_position.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

class LifecycleNavigationNode : public rclcpp_lifecycle::LifecycleNode {
    public:
        LifecycleNavigationNode(const std::string & node_name) : rclcpp_lifecycle::LifecycleNode(node_name){}

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Configuring node");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {
        RCLCPP_INFO(get_logger(), "Activating node: starting processing...");
        rclcpp_lifecycle::LifecycleNode::on_activate(state); 
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {
        RCLCPP_INFO(get_logger(), "Deactivating node: pausing processing...");
        rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Cleaning up node: freeing resources...");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Shutting down node...");
        return CallbackReturn::SUCCESS;
    }
};

int main (int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LifecycleNavigationNode>("lifecycle_navigation_node")->get_node_base_interface());
    rclcpp::shutdown();
}
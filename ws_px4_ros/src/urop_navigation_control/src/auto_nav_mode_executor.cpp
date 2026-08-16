
#include "rclcpp/rclcpp.hpp"
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <rclcpp/node.hpp>
#include <autonomous_navigation_mode.hpp>
#include <ascend_mode.hpp>

class AutoNavModeExecutor : public px4_ros2::ModeExecutorBase 
{
    public:
        AutoNavModeExecutor(px4_ros2::ModeBase &owned_mode, px4_ros2::ModeBase &ascend_mode) : ModeExecutorBase(       
            px4_ros2::ModeExecutorBase::Settings{
                px4_ros2::ModeExecutorBase::Settings::Activation::ActivateAlways
            }, 
            owned_mode),
            _auto_nav_mode(owned_mode),

            autonomous_nav_node(owned_mode.node()),
            ascend_node(ascend_mode.node()),
            _ascend_mode(ascend_mode)
            {
                RCLCPP_INFO(autonomous_nav_node.get_logger(), "initialised mode executor");
            }


        enum class State {
            arm,
            ascending,
            autoNavigation
        };
        
        void onActivate() override;
        void onDeactivate(DeactivateReason reason) override;

        void runState(State state, px4_ros2::Result result);

        px4_ros2::ModeBase &_ascend_mode;
        px4_ros2::ModeBase &_auto_nav_mode;


    
    private:
        rclcpp::Node &ascend_node;
        rclcpp::Node &autonomous_nav_node;


};

void AutoNavModeExecutor::onActivate(){
    runState(State::arm, px4_ros2::Result::Success);
}

void AutoNavModeExecutor::onDeactivate(DeactivateReason reason){}

void AutoNavModeExecutor::runState(State state, px4_ros2::Result result){
        if (result != px4_ros2::Result::Success){
            RCLCPP_INFO(autonomous_nav_node.get_logger(), "state transition failure");
            rclcpp::shutdown();
            return;
        };

        switch (state){
            case State::arm:
                    RCLCPP_INFO(autonomous_nav_node.get_logger(), "request arm" );
                arm(std::bind(&AutoNavModeExecutor::runState, this, State::ascending, std::placeholders::_1));
                break;
            case State::ascending:
                RCLCPP_INFO(autonomous_nav_node.get_logger(), "ascending" );
                scheduleMode(
                    _ascend_mode.id(), [this](px4_ros2::Result result) {
                        runState(State::autoNavigation, result);
                    });
                break;
            case State::autoNavigation:
                RCLCPP_INFO(autonomous_nav_node.get_logger(), "final mode");
                scheduleMode(_auto_nav_mode.id(), [this](px4_ros2::Result result) {
                    RCLCPP_INFO(autonomous_nav_node.get_logger(), "finished");
                });
                break;
            
        }

}

int main(int argc, char *argv[]){
    using autoNavModeExecutor = px4_ros2::NodeWithModeExecutor<AutoNavModeExecutor, AutonomousNavigationMode, AscendMode>;
    static const std::string kNodeName = "auto_nav_mode_executor";
    static const bool kEnableDebugOutput = true;
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<autoNavModeExecutor>(kNodeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}
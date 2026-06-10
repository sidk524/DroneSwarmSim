
#include "rclcpp/rclcpp.hpp"
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <fly_up.hpp>
#include <locate_marker.hpp>

class PrecisionLandingExecutor : public px4_ros2::ModeExecutorBase {
  public:
    PrecisionLandingExecutor(px4_ros2::ModeBase & owned_mode, px4_ros2::ModeBase & second_mode) : ModeExecutorBase(
      px4_ros2::ModeExecutorBase::Settings{
        px4_ros2::ModeExecutorBase::Settings::Activation::ActivateAlways
      }, 
      owned_mode
    ),
      _node(owned_mode.node()),
      _second_node(second_mode.node()),
      _second_mode(second_mode)

    { }

    enum State {
      request_arm,
      check_arm,
      taking_off,
      fly_up,
      find_marker,
      RTL,
      WaitUntilDisarmed
    };

    void onActivate() override {
      RCLCPP_DEBUG(_node.get_logger(), "hello");
        runState(State::request_arm, px4_ros2::Result::Success);
    }

    void onDeactivate(DeactivateReason reason) override {
      
    }

    void runState(State state, px4_ros2::Result previous_result){
      RCLCPP_DEBUG(_node.get_logger(), "State %i: previous state failed: %s", (int)state,
          resultToString(previous_result));

        if (state == check_arm ) {
          if (previous_result == px4_ros2::Result::Success){
            state = State::taking_off;
            
        } else{
            state = State::request_arm;
        }
      }
      
      switch (state){
          case State::request_arm:
              arm([this](px4_ros2::Result result) {runState(State::check_arm, result);});
          case State::taking_off:
            takeoff([this](px4_ros2::Result result) {runState(State::fly_up, result);}, 2.5, 1.0);
            break;
          case State::fly_up:
            scheduleMode(
              ownedMode().id(), [this](px4_ros2::Result result) {
                runState(State::find_marker, result);
            });
            break;
          case State::find_marker:
            scheduleMode(
              _second_mode.id(), [this] (px4_ros2::Result result) {
                    runState(State::RTL, result);
              }
            );
            break;
          case State::RTL:
            rtl([this](px4_ros2::Result result) {runState(State::WaitUntilDisarmed, result);});
            break;
          case State::WaitUntilDisarmed:
            waitUntilDisarmed([this](px4_ros2::Result result) {
                RCLCPP_INFO(_node.get_logger(), "All states complete (%s)", resultToString(result));
              });
            break;
      }
    }
    
    private:
      rclcpp::Node & _node;
      rclcpp::Node & _second_node;
      px4_ros2::ModeBase &_second_mode;
};



int main(int argc, char* argv[])
{
  
    using precisionLandingExecutor = px4_ros2::NodeWithModeExecutor<PrecisionLandingExecutor, InitialFlyUpMode, LocateArucoMarkerMode>;
    static const std::string kNodeName = "precision_landing_executor";
    static const bool kEnableDebugOutput = true;
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<precisionLandingExecutor>(kNodeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
  
}



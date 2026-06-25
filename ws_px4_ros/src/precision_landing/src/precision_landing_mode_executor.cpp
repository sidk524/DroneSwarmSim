
#include "rclcpp/rclcpp.hpp"
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <fly_up.hpp>
#include <locate_marker.hpp>
#include <move_above_marker.hpp>
#include <descend_mode.hpp>
#include <rclcpp/node.hpp>

class PrecisionLandingExecutor : public px4_ros2::ModeExecutorBase {
  public:
    PrecisionLandingExecutor(
      px4_ros2::ModeBase & owned_mode, px4_ros2::ModeBase & second_mode, px4_ros2::ModeBase & third_mode, px4_ros2::ModeBase & fourth_mode) : ModeExecutorBase(
      px4_ros2::ModeExecutorBase::Settings{
        px4_ros2::ModeExecutorBase::Settings::Activation::ActivateAlways
      }, 
      owned_mode
    ),
      _node(owned_mode.node()),
      _second_mode(second_mode),
      _third_mode(third_mode),
      _fourth_mode(fourth_mode),

      _second_node(second_mode.node()),
      _third_node(third_mode.node()),
      _fourth_node(fourth_mode.node())
    { }

    enum State {
      request_arm,
      check_arm,
      taking_off,
      fly_up,
      find_marker,
      move_above_marker,
      descend,
      disarmDrone
    };

    void onActivate() override {
      RCLCPP_DEBUG(_node.get_logger(), "hello");
        runState(State::request_arm, px4_ros2::Result::Success);
    }

    void onDeactivate(DeactivateReason reason) override {
      
    }

    void runState(State state, px4_ros2::Result previous_result){

        if (state == check_arm ) {
                      RCLCPP_DEBUG(_node.get_logger(), "check arm" );

          if (previous_result == px4_ros2::Result::Success){
            state = State::taking_off;
        } else{
            state = State::request_arm;
        }
      }
      switch (state){
          case State::request_arm:
              RCLCPP_INFO(_node.get_logger(), "request arm" );

              arm([this](px4_ros2::Result result) {runState(State::check_arm, result);});
              break;
          case State::taking_off:
            RCLCPP_INFO(_node.get_logger(), "taking off" );

            takeoff([this](px4_ros2::Result result) {
              if (result == px4_ros2::Result::Success){
                runState(State::fly_up, result);
              } else {
                RCLCPP_DEBUG(_node.get_logger(), "takeoff failed ");
              }
            
            }, 
            5.0, 1.0);
            break;
          case State::fly_up:
            RCLCPP_INFO(_node.get_logger(), "fly up" );

            scheduleMode(
              ownedMode().id(), [this](px4_ros2::Result result) {
                runState(State::find_marker, result);
            });

            break;
          case State::find_marker:
            RCLCPP_INFO(_node.get_logger(), "find marker");

            scheduleMode(
              _second_mode.id(), [this] (px4_ros2::Result result) {
                    runState(State::move_above_marker, result);
              }
            );
            break;
          case State::move_above_marker:
              RCLCPP_INFO(_node.get_logger(), "move above marker");

              scheduleMode(
                _third_mode.id(), [this] (px4_ros2::Result result) {
                    runState(State::descend, result);
              }
              );
            break;
          case State::descend:
            RCLCPP_INFO(_node.get_logger(), "Descend" );
              scheduleMode(
                _fourth_mode.id(), [this] (px4_ros2::Result result) {
                    runState(State::disarmDrone, result);
              }
              );
            break;
          case State::disarmDrone:
            
            disarm([this](px4_ros2::Result result) {
                RCLCPP_INFO(_node.get_logger(), "All states complete (%s)", resultToString(result));
              });
            break;
      }
    }
    private:
      rclcpp::Node & _node;
      rclcpp::Node & _second_node;
      rclcpp::Node & _third_node;
      rclcpp::Node & _fourth_node;

      px4_ros2::ModeBase &_second_mode;
      px4_ros2::ModeBase &_third_mode;
      px4_ros2::ModeBase &_fourth_mode;
};

int main(int argc, char* argv[])
{
    using precisionLandingExecutor = px4_ros2::NodeWithModeExecutor<PrecisionLandingExecutor, InitialFlyUpMode, LocateArucoMarkerMode, MoveAboveMarkerMode, DescendMode>;
    static const std::string kNodeName = "precision_landing_executor";
    static const bool kEnableDebugOutput = true;
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<precisionLandingExecutor>(kNodeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
  
}



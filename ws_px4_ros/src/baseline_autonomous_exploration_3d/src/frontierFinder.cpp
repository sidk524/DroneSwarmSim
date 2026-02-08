#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <memory>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <functional>
#include <queue>

using namespace std::chrono_literals;
using namespace octomap_msgs::msg;

class FrontierFinderNode : public rclcpp::Node
{
public:
  FrontierFinderNode();
  
  struct FrontierCluster {
    std::vector<octomap::point3d> voxels;
    octomap::point3d centroid;
    size_t size;
  };
  
  std::queue<FrontierCluster> frontier_clusters_queue;
  FrontierCluster currentGoal;

private:
  enum class VoxelState { FREE, OCCUPIED, UNKNOWN };
  
  struct VoxelKey {
    int x, y, z;
    
    bool operator==(const VoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  
  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const {
      return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1) ^ (std::hash<int>()(key.z) << 2);
    }
  };
  
  const int DRONE_VOXEL_RADIUS = 20;
  
  enum class State {
    init,
    offboard_mode_requested,
    wait_for_stable_offboard_mode,
    arm_requested,
    armed
  };
  
  void findfrontierVoxelsNearDrone();
  void octomap_callback(const Octomap::SharedPtr msg);
  VoxelState getVoxelState(const octomap::point3d& point);
  bool hasClearance(const octomap::point3d& point, double radius);
  bool isFrontier(const octomap::point3d& point);
  std::vector<FrontierCluster> findFrontierClusters(const std::vector<octomap::point3d>& frontier_voxels);
  
  void publishOffboardControlMode();
  void requestVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
  void vehicle_response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
  void armDrone();
  static const char* vehicle_cmd_result_to_string(uint8_t result);
  void publishWaypoint();
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<Octomap>::SharedPtr octomap_subscriber_;
  std::unique_ptr<octomap::AbstractOcTree> octomap_tree_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  State state_;
  uint8_t service_result_;
  bool service_done_;
  int offboard_retry_count_;
  rclcpp::Time offboard_retry_time_;
  int arm_retry_count_;
  rclcpp::Time arm_retry_time_;
};

FrontierFinderNode::FrontierFinderNode() : Node("frontier_finder_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  rclcpp::QoS qos(1);
  qos.reliable();
  qos.transient_local();

  octomap_subscriber_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "/octomap_binary",
    qos,
    std::bind(&FrontierFinderNode::octomap_callback, this, std::placeholders::_1)
  );
  
  offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
  trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
  vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");
  state_ = State::init;
  service_result_ = 0;
  service_done_ = false;
  offboard_retry_count_ = 0;
  arm_retry_count_ = 0;
  
  while (!vehicle_command_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for vehicle command service...");
  }
  
  auto timer_callback = [this]() -> void {
    publishOffboardControlMode();
    
    switch (state_) {
      case State::init:
        this->requestVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        state_ = State::offboard_mode_requested;
        break;
      case State::offboard_mode_requested:
        if (service_done_) {
          if (service_result_ == 0) {
            state_ = State::wait_for_stable_offboard_mode;
            offboard_retry_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "Offboard mode request accepted");
          } else {
            offboard_retry_count_++;
            RCLCPP_WARN(this->get_logger(),
              "Failed to set offboard mode (%s, result=%u), attempt %d/50",
              vehicle_cmd_result_to_string(service_result_),
              static_cast<unsigned>(service_result_),
              offboard_retry_count_);
            
            if (offboard_retry_count_ >= 50) {
              RCLCPP_ERROR(this->get_logger(), "Failed to set offboard mode after 50 attempts, exiting");
              rclcpp::shutdown();
            } else {
              offboard_retry_time_ = this->now() + 1s;
              state_ = State::wait_for_stable_offboard_mode;
            }
          }
        }
        break;
        
      case State::wait_for_stable_offboard_mode:
        if (offboard_retry_count_ > 0) {
          if (this->now() >= offboard_retry_time_) {
            RCLCPP_INFO(this->get_logger(), "Retrying to set offboard mode...");
            this->requestVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            state_ = State::offboard_mode_requested;
          }
        } else {
          if (arm_retry_count_ > 0) {
            if (this->now() >= arm_retry_time_) {
              RCLCPP_INFO(this->get_logger(), "Retrying to arm drone...");
              armDrone();
              state_ = State::arm_requested;
            }
          } else {
            armDrone();
            state_ = State::arm_requested;
          }
        }
        break;
        
      case State::arm_requested:
        if (service_done_) {
          if (service_result_ == 0) {
            state_ = State::armed;
            arm_retry_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "Drone armed successfully");
          } else {
            arm_retry_count_++;
            RCLCPP_WARN(this->get_logger(),
              "Failed to arm drone (%s, result=%u), attempt %d/50",
              vehicle_cmd_result_to_string(service_result_),
              static_cast<unsigned>(service_result_),
              arm_retry_count_);
            
            if (arm_retry_count_ >= 50) {
              RCLCPP_ERROR(this->get_logger(), "Failed to arm drone after 50 attempts, exiting");
              rclcpp::shutdown();
            } else {
              arm_retry_time_ = this->now() + 1s;
              state_ = State::wait_for_stable_offboard_mode;
            }
          }
        }
        break;
        
      case State::armed:
        publishWaypoint();
        break;
    }
  };
  
  timer_ = this->create_wall_timer(100ms, timer_callback);
  
  RCLCPP_INFO(this->get_logger(), "FrontierFinderNode started");
}

void FrontierFinderNode::octomap_callback(const Octomap::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received octomap with frame_id: %s", msg->header.frame_id.c_str());
  RCLCPP_WARN(this->get_logger(),
  "Octomap id: %s",
  msg->id.c_str());
  octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
  if (tree == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Failed to deserialize octomap from message");
    return;
  }
  
  octomap_tree_.reset(tree);
}

FrontierFinderNode::VoxelState FrontierFinderNode::getVoxelState(const octomap::point3d& point)
{
  if (!octomap_tree_) {
    return VoxelState::UNKNOWN;
  }

  auto* tree = dynamic_cast<octomap::ColorOcTree*>(octomap_tree_.get());
  if (!tree) {
    return VoxelState::UNKNOWN;
  }

  auto* node = tree->search(point);
  if (!node) {
    return VoxelState::UNKNOWN;
  } else if (tree->isNodeOccupied(node)) {
    return VoxelState::OCCUPIED;
  } else {
    return VoxelState::FREE;
  }
}

bool FrontierFinderNode::isFrontier(const octomap::point3d& point)
{
  if (!octomap_tree_) {
    return false;
  }

  auto* tree = dynamic_cast<octomap::ColorOcTree*>(octomap_tree_.get());
  if (!tree) {
    return false;
  }

  VoxelState point_state = getVoxelState(point);
  if (point_state != VoxelState::FREE) {
    return false;
  }

  double resolution = tree->getResolution();

  octomap::point3d neighbors[6] = {
    octomap::point3d(point.x() + resolution, point.y(), point.z()),
    octomap::point3d(point.x() - resolution, point.y(), point.z()),
    octomap::point3d(point.x(), point.y() + resolution, point.z()),
    octomap::point3d(point.x(), point.y() - resolution, point.z()),
    octomap::point3d(point.x(), point.y(), point.z() + resolution),
    octomap::point3d(point.x(), point.y(), point.z() - resolution)
  };
  for (const auto& neighbor : neighbors) {
    VoxelState neighbor_state = getVoxelState(neighbor);
    if (neighbor_state == VoxelState::UNKNOWN) {
      return true;
    }
  }

  return false;
}

bool FrontierFinderNode::hasClearance(const octomap::point3d& point, double radius)
{
  double cx = point.x();
  double cy = point.y();
  double cz = point.z();
  
  double min_x = cx - radius;
  double min_y = cy - radius;
  double min_z = cz - radius;
  
  double max_x = cx + radius;
  double max_y = cy + radius;
  double max_z = cz + radius;
  
  if (!octomap_tree_) {
    return false;
  }

  auto* tree = dynamic_cast<octomap::ColorOcTree*>(octomap_tree_.get());
  if (!tree) {
    return false;
  }

  octomap::point3d min(min_x, min_y, min_z);
  octomap::point3d max(max_x, max_y, max_z);

  for (auto it = tree->begin_leafs_bbx(min, max),
            end = tree->end_leafs_bbx();
       it != end; ++it)
  {
    octomap::point3d voxel_point = it.getCoordinate();
    
    VoxelState state = getVoxelState(voxel_point);
    
    if (state != VoxelState::FREE) {
      return false;
    }
  }
  
  return true;
}

void FrontierFinderNode::findfrontierVoxelsNearDrone()
{
  geometry_msgs::msg::TransformStamped tf;
  
  try {
    tf = tf_buffer_.lookupTransform(
      "map",
      "base_link",
      tf2::TimePointZero
    );
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }
  
  double x = tf.transform.translation.x;
  double y = tf.transform.translation.y;
  double z = tf.transform.translation.z;
  
  octomap::point3d drone_position(x, y, z);
  
  if (!octomap_tree_) {
    RCLCPP_WARN(this->get_logger(), "Octomap not available");
    return;
  }
  
  auto* tree = dynamic_cast<octomap::ColorOcTree*>(octomap_tree_.get());
  if (!tree) {
    RCLCPP_WARN(this->get_logger(), "Failed to cast octomap to OcTree");
    return;
  }
  
  double resolution = tree->getResolution();
  
  double bbox_size = DRONE_VOXEL_RADIUS * resolution;
  
  double min_x = drone_position.x() - bbox_size;
  double min_y = drone_position.y() - bbox_size;
  double min_z = drone_position.z() - bbox_size;
  
  double max_x = drone_position.x() + bbox_size;
  double max_y = drone_position.y() + bbox_size;
  double max_z = drone_position.z() + bbox_size;
  
  octomap::point3d min(min_x, min_y, min_z);
  octomap::point3d max(max_x, max_y, max_z);
  
  std::vector<octomap::point3d> current_frontier_voxels;
  
  for (auto it = tree->begin_leafs_bbx(min, max),
            end = tree->end_leafs_bbx();
       it != end; ++it)
  {
    octomap::point3d voxel_point = it.getCoordinate();
    
    if (isFrontier(voxel_point)) {
      current_frontier_voxels.push_back(voxel_point);
    }
  }
  
  std::vector<FrontierCluster> clusters = findFrontierClusters(current_frontier_voxels);
  for (const auto& cluster : clusters) {
    frontier_clusters_queue.push(cluster);
  }
}
  
std::vector<FrontierFinderNode::FrontierCluster> FrontierFinderNode::findFrontierClusters(const std::vector<octomap::point3d>& frontier_voxels)
{
  std::vector<FrontierCluster> clusters;
  
  if (!octomap_tree_) {
    RCLCPP_WARN(this->get_logger(), "Octomap not available");
    return clusters;
  }

  auto* tree = dynamic_cast<octomap::ColorOcTree*>(octomap_tree_.get());
  if (!tree) {
    RCLCPP_WARN(this->get_logger(), "Failed to cast octomap to OcTree");
    return clusters;
  }

  double r = tree->getResolution();
  
  std::unordered_set<VoxelKey, VoxelKeyHash> unvisited;
  std::unordered_map<VoxelKey, octomap::point3d, VoxelKeyHash> key_to_point;
  
  for (const auto& voxel : frontier_voxels) {
    int ix = static_cast<int>(std::floor(voxel.x() / r));
    int iy = static_cast<int>(std::floor(voxel.y() / r));
    int iz = static_cast<int>(std::floor(voxel.z() / r));
    
    VoxelKey key{ix, iy, iz};
    unvisited.insert(key);
    if (key_to_point.find(key) == key_to_point.end()) {
      key_to_point[key] = voxel;
    }
  }
  
  const std::vector<VoxelKey> neighbors = {
    {-1, -1, -1}, {-1, -1,  0}, {-1, -1,  1},
    {-1,  0, -1}, {-1,  0,  0}, {-1,  0,  1},
    {-1,  1, -1}, {-1,  1,  0}, {-1,  1,  1},
    { 0, -1, -1}, { 0, -1,  0}, { 0, -1,  1},
    { 0,  0, -1},                { 0,  0,  1},
    { 0,  1, -1}, { 0,  1,  0}, { 0,  1,  1},
    { 1, -1, -1}, { 1, -1,  0}, { 1, -1,  1},
    { 1,  0, -1}, { 1,  0,  0}, { 1,  0,  1},
    { 1,  1, -1}, { 1,  1,  0}, { 1,  1,  1}
  };
  
  while (!unvisited.empty()) {
    VoxelKey seed = *unvisited.begin();
    std::queue<VoxelKey> queue;
    queue.push(seed);
    std::vector<octomap::point3d> current_cluster;
    
    unvisited.erase(seed);
    
    while (!queue.empty()) {
      VoxelKey v = queue.front();
      queue.pop();
      
      if (key_to_point.find(v) != key_to_point.end()) {
        current_cluster.push_back(key_to_point[v]);
      }
      
      for (const auto& offset : neighbors) {
        VoxelKey n{v.x + offset.x, v.y + offset.y, v.z + offset.z};
        
        if (unvisited.find(n) != unvisited.end()) {
          unvisited.erase(n);
          queue.push(n);
        }
      }
    }
    
    if (!current_cluster.empty()) {
      FrontierCluster cluster;
      cluster.voxels = current_cluster;
      cluster.size = current_cluster.size();
      
      double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
      for (const auto& point : current_cluster) {
        sum_x += point.x();
        sum_y += point.y();
        sum_z += point.z();
      }
      cluster.centroid = octomap::point3d(
        sum_x / cluster.size,
        sum_y / cluster.size,
        sum_z / cluster.size
      );
      
      clusters.push_back(cluster);
    }
  }
  
  return clusters;
}

void FrontierFinderNode::publishOffboardControlMode()
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;
  offboard_control_mode_publisher_->publish(msg);
}

void FrontierFinderNode::requestVehicleCommand(uint16_t command, float param1, float param2)
{
  auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

  px4_msgs::msg::VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  request->request = msg;
  service_done_ = false;
  vehicle_command_client_->async_send_request(request,
    std::bind(&FrontierFinderNode::vehicle_response_callback, this, std::placeholders::_1));
}

void FrontierFinderNode::vehicle_response_callback(
  rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future)
{
  auto status = future.wait_for(1s);
  if (status == std::future_status::ready) {
    auto reply = future.get()->reply;
    service_result_ = reply.result;
    service_done_ = true;
  }
}

void FrontierFinderNode::armDrone()
{
  requestVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

const char* FrontierFinderNode::vehicle_cmd_result_to_string(uint8_t result)
{
  using px4_msgs::msg::VehicleCommandAck;
  switch (result) {
    case VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED:
      return "accepted";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
      return "temporarily rejected";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED:
      return "denied";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_UNSUPPORTED:
      return "unsupported";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_FAILED:
      return "failed";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS:
      return "in progress";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_CANCELLED:
      return "cancelled";
    default:
      return "unknown";
  }
}

void FrontierFinderNode::publishWaypoint()
{
  if (currentGoal.size == 0 || currentGoal.voxels.empty()) {
    if (!octomap_tree_) {
      RCLCPP_WARN(this->get_logger(), "Octomap not available");
      return;
    }
    findfrontierVoxelsNearDrone();
    
    if (!frontier_clusters_queue.empty()) {
      currentGoal = frontier_clusters_queue.front();
      frontier_clusters_queue.pop();
    } else {
      RCLCPP_INFO(this->get_logger(), "No more frontier clusters available");
      return;
    }
  }
  
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(
      "map",
      "base_link",
      tf2::TimePointZero
    );
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }
  
  double drone_x = tf.transform.translation.x;
  double drone_y = tf.transform.translation.y;
  double drone_z = tf.transform.translation.z;
  
  double centroid_x = currentGoal.centroid.x();
  double centroid_y = currentGoal.centroid.y();
  double centroid_z = currentGoal.centroid.z();
  
  double dx = centroid_x - drone_x;
  double dy = centroid_y - drone_y;
  double dz = centroid_z - drone_z;
  
  double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  
  if (distance < 0.1) {
    currentGoal.size = 0;
    currentGoal.voxels.clear();
    return;
  }
  
  if (!octomap_tree_) {
    RCLCPP_WARN(this->get_logger(), "Octomap not available");
    return;
  }
  
  auto* tree = dynamic_cast<octomap::ColorOcTree*>(octomap_tree_.get());
  if (!tree) {
    RCLCPP_WARN(this->get_logger(), "Failed to cast octomap to OcTree");
    return;
  }
  
  double resolution = tree->getResolution();
  
  double unit_x = dx / distance;
  double unit_y = dy / distance;
  double unit_z = dz / distance;
  
  double advance_distance = 5.0 * resolution;
  double advance_x = unit_x * advance_distance;
  double advance_y = unit_y * advance_distance;
  double advance_z = unit_z * advance_distance;
  
  double target_x = drone_x + advance_x;
  double target_y = drone_y + advance_y;
  double target_z = drone_z + advance_z;
  
  octomap::point3d target_position(target_x, target_y, target_z);
  double clearance_radius = 10.0 * resolution;
  
  if (!hasClearance(target_position, clearance_radius)) {
    frontier_clusters_queue.push(currentGoal);
    currentGoal.size = 0;
    currentGoal.voxels.clear();
    RCLCPP_WARN(this->get_logger(), "No clearance at target position, re-queuing cluster");
    return;
  }
  
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.position[0] = static_cast<float>(target_x);
  msg.position[1] = static_cast<float>(target_y);
  msg.position[2] = static_cast<float>(target_z);
  msg.yaw = -3.14f;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierFinderNode>());
  rclcpp::shutdown();
  return 0;
}


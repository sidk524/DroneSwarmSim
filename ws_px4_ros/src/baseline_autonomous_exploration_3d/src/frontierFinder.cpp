#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <memory>
#include <vector>

using namespace std::chrono_literals;
using namespace octomap_msgs::msg;

class FrontierFinderNode : public rclcpp::Node
{
public:
  FrontierFinderNode();

private:
  enum class VoxelState { FREE, OCCUPIED, UNKNOWN };
  
  const int DRONE_VOXEL_RADIUS = 20;  // drone voxel radius
  void findfrontierVoxelsNearDrone();
  void octomap_callback(const Octomap::SharedPtr msg);
  VoxelState getVoxelState(const octomap::point3d& point);
  bool hasClearance(const octomap::point3d& point, double radius);
  bool isFrontier(const octomap::point3d& point);
  void findFrontierClusters(const std::vector<octomap::point3d>& frontier_voxels);
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<Octomap>::SharedPtr octomap_subscriber_;
  std::unique_ptr<octomap::AbstractOcTree> octomap_tree_;
};

FrontierFinderNode::FrontierFinderNode() : Node("frontier_finder_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  octomap_subscriber_{this->create_subscription<Octomap>(
    "/octomap_binary",
    10,
    std::bind(&FrontierFinderNode::octomap_callback, this, std::placeholders::_1)
  )}
{
  RCLCPP_INFO(this->get_logger(), "FrontierFinderNode started");
}

void FrontierFinderNode::octomap_callback(const Octomap::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received octomap with frame_id: %s", msg->header.frame_id.c_str());
  
  // Deserialize the octomap from the message
  octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
  if (tree == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Failed to deserialize octomap from message");
    return;
  }
  
  // Update the tree variable with the latest map
  octomap_tree_.reset(tree);
}

FrontierFinderNode::VoxelState FrontierFinderNode::getVoxelState(const octomap::point3d& point)
{
  if (!octomap_tree_) {
    return VoxelState::UNKNOWN;
  }

  // Cast to OcTree to access search and isNodeOccupied methods
  auto* tree = dynamic_cast<octomap::OcTree*>(octomap_tree_.get());
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

  // Cast to OcTree to access resolution
  auto* tree = dynamic_cast<octomap::OcTree*>(octomap_tree_.get());
  if (!tree) {
    return false;
  }

  // Check if the point itself is FREE
  VoxelState point_state = getVoxelState(point);
  if (point_state != VoxelState::FREE) {
    return false;
  }

  // Get the voxel resolution for neighbor checking
  double resolution = tree->getResolution();

  // Check all 6 immediate neighbors (±1 in each direction)
  // Neighbors: +x, -x, +y, -y, +z, -z
  octomap::point3d neighbors[6] = {
    octomap::point3d(point.x() + resolution, point.y(), point.z()),           // +x
    octomap::point3d(point.x() - resolution, point.y(), point.z()),           // -x
    octomap::point3d(point.x(), point.y() + resolution, point.z()),           // +y
    octomap::point3d(point.x(), point.y() - resolution, point.z()),           // -y
    octomap::point3d(point.x(), point.y(), point.z() + resolution),           // +z
    octomap::point3d(point.x(), point.y(), point.z() - resolution)            // -z
  };

  // Check if at least one neighbor is UNKNOWN
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
  // Step 1: Define the region in metric space
  // Extract center coordinates
  double cx = point.x();
  double cy = point.y();
  double cz = point.z();
  
  // Define an axis-aligned bounding box (AABB)
  double min_x = cx - radius;
  double min_y = cy - radius;
  double min_z = cz - radius;
  
  double max_x = cx + radius;
  double max_y = cy + radius;
  double max_z = cz + radius;
  
  // Step 2: Use OctoMap's bounded leaf iterator
  if (!octomap_tree_) {
    return false;
  }

  // Cast to OcTree to access iterator methods
  auto* tree = dynamic_cast<octomap::OcTree*>(octomap_tree_.get());
  if (!tree) {
    return false;
  }

  // Create min and max point3d objects for the bounding box
  octomap::point3d min(min_x, min_y, min_z);
  octomap::point3d max(max_x, max_y, max_z);

  // Iterate over leaf nodes within the bounding box
  for (auto it = tree->begin_leafs_bbx(min, max),
            end = tree->end_leafs_bbx();
       it != end; ++it)
  {
    // Get the coordinate of the current leaf node
    octomap::point3d voxel_point = it.getCoordinate();
    
    // Check the voxel state
    VoxelState state = getVoxelState(voxel_point);
    
    // If any voxel is not FREE, return false immediately
    if (state != VoxelState::FREE) {
      return false;
    }
  }
  
  return true;
}

void FrontierFinderNode::findfrontierVoxelsNearDrone()
{
  // Query the transform from map to base_link
  geometry_msgs::msg::TransformStamped tf;
  
  try {
    tf = tf_buffer_.lookupTransform(
      "map",        // target frame
      "base_link",  // source frame
      tf2::TimePointZero
    );
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }
  
  // Extract position
  double x = tf.transform.translation.x;
  double y = tf.transform.translation.y;
  double z = tf.transform.translation.z;
  
  // Convert to octomap Point3D
  octomap::point3d drone_position(x, y, z);
  
  // Check if octomap is available
  if (!octomap_tree_) {
    RCLCPP_WARN(this->get_logger(), "Octomap not available");
    return;
  }
  
  // Cast to OcTree to access resolution and iterator methods
  auto* tree = dynamic_cast<octomap::OcTree*>(octomap_tree_.get());
  if (!tree) {
    RCLCPP_WARN(this->get_logger(), "Failed to cast octomap to OcTree");
    return;
  }
  
  // Get the voxel resolution
  double resolution = tree->getResolution();
  
  // Calculate bounding box size: DRONE_VOXEL_RADIUS × resolution
  double bbox_size = DRONE_VOXEL_RADIUS * resolution;
  
  // Define the bounding box centered on drone position
  double min_x = drone_position.x() - bbox_size;
  double min_y = drone_position.y() - bbox_size;
  double min_z = drone_position.z() - bbox_size;
  
  double max_x = drone_position.x() + bbox_size;
  double max_y = drone_position.y() + bbox_size;
  double max_z = drone_position.z() + bbox_size;
  
  // Create min and max point3d objects for the bounding box
  octomap::point3d min(min_x, min_y, min_z);
  octomap::point3d max(max_x, max_y, max_z);
  
  // Vector to store frontier voxels
  std::vector<octomap::point3d> current_frontier_voxels;
  
  // Iterate over leaf nodes within the bounding box
  for (auto it = tree->begin_leafs_bbx(min, max),
            end = tree->end_leafs_bbx();
       it != end; ++it)
  {
    // Get the coordinate of the current leaf node
    octomap::point3d voxel_point = it.getCoordinate();
    
    // Check if this voxel is a frontier
    if (isFrontier(voxel_point)) {
      current_frontier_voxels.push_back(voxel_point);
    }
  }
}

void FrontierFinderNode::findFrontierClusters(const std::vector<octomap::point3d>& frontier_voxels)
{
  // Dummy function - to be implemented
  (void)frontier_voxels;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierFinderNode>());
  rclcpp::shutdown();
  return 0;
}


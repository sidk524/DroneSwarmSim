#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>
#include <optional>
#include <cmath>

using namespace sensor_msgs::msg;
using namespace std::chrono_literals;

class LidarDeskewedAdapter : public rclcpp::Node
{
public:
  LidarDeskewedAdapter() : Node("lidarDeskewedAdapter"),
    lidar_points_subscriber_{this->create_subscription<PointCloud2>(
      "/lidar/points",
      10,
      std::bind(&LidarDeskewedAdapter::lidar_points_callback, this, std::placeholders::_1)
    )},
    lidar_points_deskewed_publisher_{this->create_publisher<PointCloud2>(
      "/lidar/points_deskewed",
      10
    )}
  {
    RCLCPP_INFO(this->get_logger(), "LidarDeskewedAdapter node started");
  }

private:
  static std::optional<PointField> find_field(const PointCloud2& msg, const std::string& name)
  {
    for (const auto& f : msg.fields) {
      if (f.name == name) return f;
    }
    return std::nullopt;
  }

  template <typename T>
  static T read_at(const uint8_t* base, uint32_t offset)
  {
    T v;
    std::memcpy(&v, base + offset, sizeof(T));
    return v;
  }

  template <typename T>
  static void write_at(uint8_t* base, uint32_t offset, const T& v)
  {
    std::memcpy(base + offset, &v, sizeof(T));
  }

  static PointField make_field(const std::string& name, uint32_t offset, uint8_t datatype, uint32_t count = 1)
  {
    PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = datatype;
    f.count = count;
    return f;
  }

  void lidar_points_callback(const PointCloud2::SharedPtr msg)
  {
    const auto fx = find_field(*msg, "x");
    const auto fy = find_field(*msg, "y");
    const auto fz = find_field(*msg, "z");

    if (!fx || !fy || !fz) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Input cloud missing x/y/z fields. Can't adapt for LIO-SAM.");
      return;
    }

    // We assume Gazebo publishes x/y/z as float32
    if (fx->datatype != PointField::FLOAT32 || fy->datatype != PointField::FLOAT32 || fz->datatype != PointField::FLOAT32) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Input cloud x/y/z are not FLOAT32. Adapt this code if needed.");
      return;
    }

    const uint32_t num_points = msg->width * msg->height;
    if (num_points == 0) return;

    // ---- Parameters you should eventually make ROS params ----
    const uint16_t N_SCAN = 64;         // vertical beams (matches your 3D lidar config)
    const float scan_rate_hz = 20.0f;   // lidar scan rate (matches your update_rate)
    const float scan_period = 1.0f / scan_rate_hz;
    // ----------------------------------------------------------

    PointCloud2 out;
    out.header = msg->header;          // keep timestamp + frame_id
    out.height = 1;
    out.width = num_points;
    out.is_bigendian = msg->is_bigendian;
    out.is_dense = msg->is_dense;

    // LIO-SAM-friendly layout: x y z intensity ring time
    // Offsets: 0,4,8,12,16,20; point_step = 24
    out.fields.clear();
    out.fields.push_back(make_field("x",         0,  PointField::FLOAT32));
    out.fields.push_back(make_field("y",         4,  PointField::FLOAT32));
    out.fields.push_back(make_field("z",         8,  PointField::FLOAT32));
    out.fields.push_back(make_field("intensity", 12, PointField::FLOAT32));
    out.fields.push_back(make_field("ring",      16, PointField::UINT16));
    // 2 bytes padding implicitly between ring(2) and time(4) by using offset 20
    out.fields.push_back(make_field("time",      20, PointField::FLOAT32));

    out.point_step = 24;
    out.row_step = out.point_step * out.width;
    out.data.resize(out.row_step * out.height);

    const uint8_t* in_data = msg->data.data();
    uint8_t* out_data = out.data.data();

    for (uint32_t i = 0; i < num_points; ++i) {
      const uint8_t* in_pt  = in_data  + i * msg->point_step;
      uint8_t*       out_pt = out_data + i * out.point_step;

      const float x = read_at<float>(in_pt, fx->offset);
      const float y = read_at<float>(in_pt, fy->offset);
      const float z = read_at<float>(in_pt, fz->offset);

      // Basic NaN handling (optional but good)
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        // If you want, you can set is_dense=false and write zeros; for now just write zeros.
        write_at<float>(out_pt, 0, 0.0f);
        write_at<float>(out_pt, 4, 0.0f);
        write_at<float>(out_pt, 8, 0.0f);
      } else {
        write_at<float>(out_pt, 0, x);
        write_at<float>(out_pt, 4, y);
        write_at<float>(out_pt, 8, z);
      }

      const float intensity = 1.0f;                // dummy
      const uint16_t ring   = static_cast<uint16_t>(i % N_SCAN);
      const float rel_time  = (static_cast<float>(i) / static_cast<float>(num_points)) * scan_period;

      write_at<float>(out_pt, 12, intensity);
      write_at<uint16_t>(out_pt, 16, ring);

      // padding bytes at 18-19 left as whatever; set to 0 for cleanliness:
      out_pt[18] = 0;
      out_pt[19] = 0;

      write_at<float>(out_pt, 20, rel_time);
    }

    lidar_points_deskewed_publisher_->publish(out);

    RCLCPP_DEBUG(this->get_logger(),
                 "Adapted %u points -> /lidar/points_deskewed (point_step=%u)",
                 num_points, out.point_step);
  }

  rclcpp::Subscription<PointCloud2>::SharedPtr lidar_points_subscriber_;
  rclcpp::Publisher<PointCloud2>::SharedPtr lidar_points_deskewed_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDeskewedAdapter>());
  rclcpp::shutdown();
  return 0;
}
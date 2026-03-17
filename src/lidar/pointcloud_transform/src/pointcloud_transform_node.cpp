/**
 * 点云 TF 变换节点：将倾斜安装的雷达点云变换到水平坐标系
 * 订阅 /livox/lidar (livox_frame)，变换到 livox_frame_ground，发布到 /livox/lidar_corrected
 * 需配合 static_transform_publisher: livox_frame -> livox_frame_ground (pitch -50°)
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class PointcloudTransformNode : public rclcpp::Node {
public:
  PointcloudTransformNode() : Node("pointcloud_transform_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
    this->declare_parameter<std::string>("source_frame", "livox_frame");
    this->declare_parameter<std::string>("target_frame", "livox_frame_ground");
    this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    this->declare_parameter<std::string>("output_topic", "/livox/lidar_corrected");

    source_frame_ = this->get_parameter("source_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, rclcpp::SensorDataQoS(),
        std::bind(&PointcloudTransformNode::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

    RCLCPP_INFO(this->get_logger(), "Pointcloud transform: %s -> %s", source_frame_.c_str(), target_frame_.c_str());
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (msg->header.frame_id != source_frame_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Input frame_id '%s' != source_frame '%s'", msg->header.frame_id.c_str(), source_frame_.c_str());
    }
    try {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
          target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
      sensor_msgs::msg::PointCloud2 out;
      tf2::doTransform(*msg, out, transform);
      out.header.frame_id = target_frame_;
      out.header.stamp = msg->header.stamp;  // 保留原始时间戳
      pub_->publish(out);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "TF lookup failed: %s", ex.what());
    }
  }

  std::string source_frame_;
  std::string target_frame_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudTransformNode>());
  rclcpp::shutdown();
  return 0;
}

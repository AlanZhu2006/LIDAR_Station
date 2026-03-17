#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include <numeric>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/logging.hpp>
#include <vector>
#include <chrono>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp_components/register_node_macro.hpp>
namespace tdt_radar{
class Cluster : public rclcpp::Node
{
    public:
    Cluster(const rclcpp::NodeOptions& node_options);
    ~Cluster(){}
    
    private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    double cluster_tolerance_ = 0.25;
    int min_cluster_size_ = 12;
    int max_cluster_size_ = 1000;
    double cluster_voxel_leaf_size_ = 0.0;
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;
};
}//namespace tdt_radar

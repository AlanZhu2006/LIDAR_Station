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
    int min_cluster_size_ = 6;
    int max_cluster_size_ = 1000;
    double cluster_voxel_leaf_size_ = 0.0;
    double min_cluster_diagonal_ = 0.08;
    double max_cluster_diagonal_ = 2.50;
    double min_cluster_height_ = 0.03;
    double max_cluster_height_ = 2.20;
    double min_cluster_xy_span_ = 0.04;
    bool enable_small_cluster_recovery_ = true;
    int small_cluster_min_size_ = 4;
    double small_cluster_min_diagonal_ = 0.05;
    double small_cluster_max_diagonal_ = 1.20;
    double small_cluster_min_height_ = 0.04;
    double small_cluster_max_height_ = 1.20;
    double small_cluster_min_xy_span_ = 0.03;
    bool cluster_use_bbox_center_ = false;
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;
};
}//namespace tdt_radar

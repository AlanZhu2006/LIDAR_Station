#include "cluster.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <rclcpp/duration.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <algorithm>
#include <cmath>
namespace tdt_radar{

    Cluster::Cluster(const rclcpp::NodeOptions& node_options): Node("cluster_node", node_options)
    {
        RCLCPP_INFO(this->get_logger(), "cluster_node start");
        this->declare_parameter<double>("cluster_tolerance", 0.25);
        this->declare_parameter<int>("min_cluster_size", 6);
        this->declare_parameter<int>("max_cluster_size", 1000);
        this->declare_parameter<double>("cluster_voxel_leaf_size", 0.0);
        this->declare_parameter<double>("min_cluster_diagonal", 0.08);
        this->declare_parameter<double>("max_cluster_diagonal", 2.50);
        this->declare_parameter<double>("min_cluster_height", 0.03);
        this->declare_parameter<double>("max_cluster_height", 2.20);
        this->declare_parameter<double>("min_cluster_xy_span", 0.04);
        this->declare_parameter<bool>("enable_small_cluster_recovery", true);
        this->declare_parameter<int>("small_cluster_min_size", 4);
        this->declare_parameter<double>("small_cluster_min_diagonal", 0.05);
        this->declare_parameter<double>("small_cluster_max_diagonal", 1.20);
        this->declare_parameter<double>("small_cluster_min_height", 0.04);
        this->declare_parameter<double>("small_cluster_max_height", 1.20);
        this->declare_parameter<double>("small_cluster_min_xy_span", 0.03);
        this->declare_parameter<bool>("cluster_use_bbox_center", false);
        this->get_parameter("cluster_tolerance", cluster_tolerance_);
        this->get_parameter("min_cluster_size", min_cluster_size_);
        this->get_parameter("max_cluster_size", max_cluster_size_);
        this->get_parameter("cluster_voxel_leaf_size", cluster_voxel_leaf_size_);
        this->get_parameter("min_cluster_diagonal", min_cluster_diagonal_);
        this->get_parameter("max_cluster_diagonal", max_cluster_diagonal_);
        this->get_parameter("min_cluster_height", min_cluster_height_);
        this->get_parameter("max_cluster_height", max_cluster_height_);
        this->get_parameter("min_cluster_xy_span", min_cluster_xy_span_);
        this->get_parameter("enable_small_cluster_recovery", enable_small_cluster_recovery_);
        this->get_parameter("small_cluster_min_size", small_cluster_min_size_);
        this->get_parameter("small_cluster_min_diagonal", small_cluster_min_diagonal_);
        this->get_parameter("small_cluster_max_diagonal", small_cluster_max_diagonal_);
        this->get_parameter("small_cluster_min_height", small_cluster_min_height_);
        this->get_parameter("small_cluster_max_height", small_cluster_max_height_);
        this->get_parameter("small_cluster_min_xy_span", small_cluster_min_xy_span_);
        this->get_parameter("cluster_use_bbox_center", cluster_use_bbox_center_);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar_dynamic",
            rclcpp::SensorDataQoS(),
            std::bind(&Cluster::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_cluster", rclcpp::SensorDataQoS());
        RCLCPP_INFO(
            this->get_logger(),
            "Cluster params: tolerance=%.2f, min=%d, max=%d, voxel=%.2f, diag=[%.2f,%.2f], h=[%.2f,%.2f], xy_span>=%.2f, small_recovery=%s(size>=%d, diag=[%.2f,%.2f], h=[%.2f,%.2f], xy_span>=%.2f), bbox_center=%s",
            cluster_tolerance_, min_cluster_size_, max_cluster_size_, cluster_voxel_leaf_size_,
            min_cluster_diagonal_, max_cluster_diagonal_, min_cluster_height_, max_cluster_height_,
            min_cluster_xy_span_, enable_small_cluster_recovery_ ? "true" : "false",
            small_cluster_min_size_, small_cluster_min_diagonal_, small_cluster_max_diagonal_,
            small_cluster_min_height_, small_cluster_max_height_, small_cluster_min_xy_span_,
            cluster_use_bbox_center_ ? "true" : "false");
    }


void Cluster::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Cluster: lidar_dynamic empty, skip");
        return;
    }
    if (cluster_voxel_leaf_size_ > 0.0) {
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        float leaf = static_cast<float>(cluster_voxel_leaf_size_);
        sor.setLeafSize(leaf, leaf, leaf);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*downsampled);
        cloud = downsampled;
    }
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    auto time = std::chrono::system_clock::now();

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    int extraction_min_cluster_size = min_cluster_size_;
    if (enable_small_cluster_recovery_) {
        extraction_min_cluster_size =
            std::max(1, std::min(min_cluster_size_, small_cluster_min_size_));
    }
    ec.setMinClusterSize(extraction_min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract (cluster_indices);
    // std::cout<<(std::chrono::system_clock::now()-time).count()<<"ms"<<std::endl;
    
    pcl::PointCloud<pcl::PointXYZ> out_cloud;
    int rejected_shape = 0;
    int rejected_small = 0;
    int recovered_small = 0;
    auto meets_shape = [](float diagonal,
                          float dz,
                          float xy_span,
                          double min_diagonal,
                          double max_diagonal,
                          double min_height,
                          double max_height,
                          double min_xy_span) {
        return diagonal >= min_diagonal &&
               diagonal <= max_diagonal &&
               dz >= min_height &&
               dz <= max_height &&
               xy_span >= min_xy_span;
    };
    for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for(auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }        
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::PointXYZ min_pt;
        pcl::PointXYZ max_pt;
        pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
        float dx = max_pt.x - min_pt.x;
        float dy = max_pt.y - min_pt.y;
        float dz = max_pt.z - min_pt.z;
        float diagonal = std::sqrt(dx * dx + dy * dy);
        float xy_span = std::max(dx, dy);
        const int cluster_size = static_cast<int>(cloud_cluster->points.size());
        const bool meets_standard_shape =
            meets_shape(diagonal,
                        dz,
                        xy_span,
                        min_cluster_diagonal_,
                        max_cluster_diagonal_,
                        min_cluster_height_,
                        max_cluster_height_,
                        min_cluster_xy_span_);
        const bool recovered_small_cluster =
            enable_small_cluster_recovery_ &&
            cluster_size < min_cluster_size_ &&
            cluster_size >= small_cluster_min_size_ &&
            meets_shape(diagonal,
                        dz,
                        xy_span,
                        small_cluster_min_diagonal_,
                        small_cluster_max_diagonal_,
                        small_cluster_min_height_,
                        small_cluster_max_height_,
                        small_cluster_min_xy_span_);

        if (!meets_standard_shape && !recovered_small_cluster) {
            if (cluster_size < min_cluster_size_) {
                rejected_small++;
            } else {
                rejected_shape++;
            }
            continue;
        }

        if (recovered_small_cluster) {
            recovered_small++;
        }

        pcl::PointXYZ move_point;
        if (cluster_use_bbox_center_) {
            move_point.x = (min_pt.x + max_pt.x) * 0.5f;
            move_point.y = (min_pt.y + max_pt.y) * 0.5f;
            move_point.z = (min_pt.z + max_pt.z) * 0.5f;
        } else {
            move_point.x = 0.0f;
            move_point.y = 0.0f;
            move_point.z = 0.0f;
            for(auto point:cloud_cluster->points)
            {
                move_point.x += point.x;
                move_point.y += point.y;
                move_point.z += point.z;
            }
            move_point.x /= cloud_cluster->points.size();
            move_point.y /= cloud_cluster->points.size();
            move_point.z /= cloud_cluster->points.size();
        }
        out_cloud.points.push_back(move_point);
    }
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(out_cloud, output);
    output.header.frame_id = "rm_frame";
    output.header.stamp = msg->header.stamp;
    pub_->publish(output);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Cluster: in=%zu -> raw=%zu -> out=%zu clusters, recovered_small=%d, rejected_shape=%d, rejected_small=%d, %f ms",
        cloud->size(), cluster_indices.size(), out_cloud.size(), recovered_small, rejected_shape, rejected_small,
        std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()/1000.0);
}
}//namespace tdt_radar

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Cluster)

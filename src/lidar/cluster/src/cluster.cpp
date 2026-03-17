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
        this->declare_parameter<int>("min_cluster_size", 8);
        this->declare_parameter<int>("max_cluster_size", 1000);
        this->declare_parameter<double>("cluster_voxel_leaf_size", 0.0);
        this->declare_parameter<double>("min_cluster_diagonal", 0.08);
        this->declare_parameter<double>("max_cluster_diagonal", 2.50);
        this->declare_parameter<double>("min_cluster_height", 0.03);
        this->declare_parameter<double>("max_cluster_height", 2.20);
        this->declare_parameter<double>("min_cluster_xy_span", 0.04);
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
        this->get_parameter("cluster_use_bbox_center", cluster_use_bbox_center_);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar_dynamic",
            rclcpp::SensorDataQoS(),
            std::bind(&Cluster::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_cluster", rclcpp::SensorDataQoS());
        RCLCPP_INFO(
            this->get_logger(),
            "Cluster params: tolerance=%.2f, min=%d, max=%d, voxel=%.2f, diag=[%.2f,%.2f], h=[%.2f,%.2f], xy_span>=%.2f, bbox_center=%s",
            cluster_tolerance_, min_cluster_size_, max_cluster_size_, cluster_voxel_leaf_size_,
            min_cluster_diagonal_, max_cluster_diagonal_, min_cluster_height_, max_cluster_height_,
            min_cluster_xy_span_, cluster_use_bbox_center_ ? "true" : "false");
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
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract (cluster_indices);
    // std::cout<<(std::chrono::system_clock::now()-time).count()<<"ms"<<std::endl;
    
    pcl::PointCloud<pcl::PointXYZ> out_cloud;
    int rejected_shape = 0;
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
        if (diagonal < min_cluster_diagonal_ ||
            diagonal > max_cluster_diagonal_ ||
            dz < min_cluster_height_ ||
            dz > max_cluster_height_ ||
            xy_span < min_cluster_xy_span_) {
            rejected_shape++;
            continue;
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
        "Cluster: in=%zu -> raw=%zu -> out=%zu clusters, rejected_shape=%d, %f ms",
        cloud->size(), cluster_indices.size(), out_cloud.size(), rejected_shape,
        std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()/1000.0);
}
}//namespace tdt_radar

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Cluster)

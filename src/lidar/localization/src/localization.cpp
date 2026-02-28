#include <iostream>
#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace tdt_radar {
    struct Grid {
  pcl::PointXYZ farthestPoint;
  double maxDistance = -1.0;
};
class Localization : public rclcpp::Node {
public:
    Localization(const rclcpp::NodeOptions& node_options) : Node("localization", node_options) {
        std::string target_pcd_file = "config/RM2024.pcd";
        // 从pcd读取场地点云
        target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile(target_pcd_file, *target_cloud_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load %s", target_pcd_file.c_str());
            return;
        }

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10, std::bind(&Localization::callback, this, std::placeholders::_1));

        // 发布场地点云到 /livox/map 话题
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/map", 10);
        filter_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filter_map", 10);
        
        //为map_pub设置计时器
        timer_ = this->create_wall_timer(std::chrono::seconds(10), [this]() {
            sensor_msgs::msg::PointCloud2 target_msg;
            pcl::toROSMsg(*target_cloud_, target_msg);
            target_msg.header.frame_id = "rm_frame";
            publisher_->publish(target_msg);
        });

        // 初始化TF广播（普通和静态）
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // 高频发布静态TF定时器 (10Hz)
        tf_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
            if (has_aligned_ || transform != Eigen::Matrix4f::Identity()) {
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped.header.stamp = rclcpp::Time(0);  // 静态TF使用时间0
                transform_stamped.header.frame_id = "rm_frame";
                transform_stamped.child_frame_id = "livox_frame";
                transform_stamped.transform.translation.x = transform(0, 3);
                transform_stamped.transform.translation.y = transform(1, 3);
                transform_stamped.transform.translation.z = transform(2, 3);
                Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
                Eigen::Quaternionf q(rotation);
                transform_stamped.transform.rotation.x = q.x();
                transform_stamped.transform.rotation.y = q.y();
                transform_stamped.transform.rotation.z = q.z();
                transform_stamped.transform.rotation.w = q.w();
                static_tf_broadcaster_->sendTransform(transform_stamped);
            }
        });
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if(!has_aligned_){

        // 从消息中转换 source_cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *source_cloud);
        if(accumulated_clouds_.size() < accumulate_time){
            accumulated_clouds_.push_back(source_cloud);
            return;
        }else{
            accumulated_clouds_.erase(accumulated_clouds_.begin());
            accumulated_clouds_.push_back(source_cloud);
        }
        //对于每个点云，首先输入到GirdMap中，保存每个栅格中的最远点
        for(auto accumulated_cloud : accumulated_clouds_){
            for (const auto& point : *accumulated_cloud)
                {
                double azimuth = std::atan2(point.y, point.x);
                double elevation = std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y));

                int azimuthIndex = static_cast<int>(floor(azimuth * 180.0 / M_PI / gridSizeDegrees));
                int elevationIndex = static_cast<int>(floor(elevation * 180.0 / M_PI / gridSizeDegrees));

                double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

                auto& grid = gridMap[std::make_pair(azimuthIndex, elevationIndex)];
                if (distance > grid.maxDistance)
                {
                    grid.farthestPoint = point;
                    grid.maxDistance = distance;
                }
            }
        }
        //然后把gripMap中的数据转换为点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& item : gridMap)
        {
            if (item.second.maxDistance > 0.0)
            {
                result->push_back(item.second.farthestPoint);
            }
        }

        //取点云（放宽范围以适应不同场地）
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for(auto point : result->points){
            // 原条件针对RM2024场地，现在放宽为更大范围
            if(point.x > -50 && point.x < 50 && point.y > -50 && point.y < 50 && point.z < 20){
                final_cloud->push_back(point);
            }
        }
  
        // 下采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

        voxelgrid.setInputCloud(target_cloud_);
        voxelgrid.filter(*downsampled);
        *target_cloud_ = *downsampled;

        voxelgrid.setInputCloud(final_cloud);
        voxelgrid.filter(*downsampled);
        source_cloud = downsampled;

        sensor_msgs::msg::PointCloud2 filter_msg;
        pcl::toROSMsg(*source_cloud, filter_msg);
        filter_msg.header.frame_id = "livox_frame";
        filter_publisher_->publish(filter_msg);

        // 进行点云配准
        if (target_cloud_->empty() || source_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Skipping GICP: target_cloud size=%zu, source_cloud size=%zu", 
                        target_cloud_->size(), source_cloud->size());
            return;
        }
        std::cout << "--- pcl::GICP ---" << std::endl;
        boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
        transform = align(gicp, target_cloud_, source_cloud);
        }
        publishTF(transform, msg->header.stamp);
    }

    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 align(boost::shared_ptr<pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>> registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud) {
        registration->setInputTarget(target_cloud);
        registration->setInputSource(source_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
        auto t1 = std::chrono::system_clock::now();
        registration->align(*aligned);
        auto t2 = std::chrono::system_clock::now();
        // std::cout << "calib time   : " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "[msec]" << std::endl;
        RCLCPP_WARN(this->get_logger(), "calib result : %f", registration->getFitnessScore());

        if(registration->getFitnessScore()<0.1){
        has_aligned_ = true;}

        //打印变换矩阵
        return registration->getFinalTransformation();
    }

    void publishTF(const Eigen::Matrix4f& transform, const builtin_interfaces::msg::Time& stamp) {

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = "rm_frame";
        transform_stamped.child_frame_id = "livox_frame";
        transform_stamped.transform.translation.x = transform(0, 3);
        transform_stamped.transform.translation.y = transform(1, 3);
        transform_stamped.transform.translation.z = transform(2, 3);
        Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rotation);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(transform_stamped);
    }
    
    bool has_aligned_ = false;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    std::string target_pcd_file_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    //创建一个数组用来积分点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;
    int accumulate_time = 20;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filter_publisher_;
    //为map_pub设置计时器
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    double gridSizeDegrees=0.1;//0.1°*0.1°
    std::map<std::pair<int, int>, Grid> gridMap;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};
} // namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Localization)
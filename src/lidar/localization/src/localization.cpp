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
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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
    Localization(const rclcpp::NodeOptions& node_options)
        : Node("localization", node_options),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_) {
        this->declare_parameter<std::string>("map_file", "config/RM2024.pcd");
        this->declare_parameter<std::string>("point_cloud_topic", "/livox/lidar");
        this->declare_parameter<std::string>("tf_child_frame", "livox_frame");
        std::string target_pcd_file;
        std::string point_cloud_topic;
        this->get_parameter("map_file", target_pcd_file);
        this->get_parameter("point_cloud_topic", point_cloud_topic);
        this->get_parameter("tf_child_frame", tf_child_frame_);
        // 从pcd读取场地点云
        target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile(target_pcd_file, *target_cloud_) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load %s", target_pcd_file.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded map: %s (%zu points)", target_pcd_file.c_str(), target_cloud_->size());

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic, 10, std::bind(&Localization::callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Localization input topic: %s", point_cloud_topic.c_str());

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

        // TF 广播：仅用 dynamic，配准后发布；静态 identity 由 lidar.launch 的 static_transform_publisher 提供
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!msg->header.frame_id.empty()) {
            input_frame_id_ = msg->header.frame_id;
        }
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
        filter_msg.header.frame_id = input_frame_id_;
        filter_publisher_->publish(filter_msg);

        // 进行点云配准
        if (target_cloud_->empty() || source_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Skipping GICP: target_cloud size=%zu, source_cloud size=%zu", 
                        target_cloud_->size(), source_cloud->size());
            return;
        }
        boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
        gicp->setMaximumIterations(35);           // 50->35 性能优化
        gicp->setTransformationEpsilon(1e-5);     // 1e-6->1e-5 收敛阈值
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
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "GICP calib result: %f", registration->getFitnessScore());

        if(registration->getFitnessScore()<0.1){
        has_aligned_ = true;}

        //打印变换矩阵
        return registration->getFinalTransformation();
    }

    void publishTF(const Eigen::Matrix4f& transform, const builtin_interfaces::msg::Time& stamp) {
        Eigen::Matrix4f tf_to_child = transform;
        if (input_frame_id_ != tf_child_frame_) {
            try {
                auto input_to_child = tf_buffer_.lookupTransform(
                    input_frame_id_, tf_child_frame_, rclcpp::Time(0),
                    rclcpp::Duration::from_seconds(0.2));
                Eigen::Affine3f composed = Eigen::Affine3f::Identity();
                composed.translation() << input_to_child.transform.translation.x,
                                          input_to_child.transform.translation.y,
                                          input_to_child.transform.translation.z;
                Eigen::Quaternionf q_child(
                    input_to_child.transform.rotation.w,
                    input_to_child.transform.rotation.x,
                    input_to_child.transform.rotation.y,
                    input_to_child.transform.rotation.z);
                composed.rotate(q_child);
                tf_to_child = transform * composed.matrix();
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Compose TF failed (%s <- %s): %s",
                    input_frame_id_.c_str(), tf_child_frame_.c_str(), ex.what());
            }
        }

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = "rm_frame";
        transform_stamped.child_frame_id = tf_child_frame_;
        transform_stamped.transform.translation.x = tf_to_child(0, 3);
        transform_stamped.transform.translation.y = tf_to_child(1, 3);
        transform_stamped.transform.translation.z = tf_to_child(2, 3);
        Eigen::Matrix3f rotation = tf_to_child.block<3, 3>(0, 0);
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
    int accumulate_time = 10;  // 10帧后开始GICP，减少可加快初始对齐
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filter_publisher_;
    //为map_pub设置计时器
    rclcpp::TimerBase::SharedPtr timer_;
    double gridSizeDegrees=0.1;//0.1°*0.1°
    std::map<std::pair<int, int>, Grid> gridMap;
    std::string input_frame_id_ = "livox_frame";
    std::string tf_child_frame_ = "livox_frame";

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
} // namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Localization)

#pragma once

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "filter_plus.h"
#include <rclcpp/publisher.hpp>
#include <vision_interface/msg/detect_result.hpp>
#include <vision_interface/msg/radar2_sentry.hpp>
#include <vision_interface/msg/radar_warn.hpp>
#include <vision_interface/msg/match_info.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <mutex>

namespace tdt_radar{
class KalmanFilter :public rclcpp::Node
{
    public:
    KalmanFilter(const rclcpp::NodeOptions& node_options);
    ~KalmanFilter(){}
    
    private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr sub_detect_;
    rclcpp::Subscription<vision_interface::msg::RadarWarn>::SharedPtr sub_lidar_;
    rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr sub_match_;
    rclcpp::Publisher<vision_interface::msg::Radar2Sentry>::SharedPtr radar_pub_;
    rclcpp::Publisher<vision_interface::msg::DetectResult>::SharedPtr radar_detect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void detect_callback(const vision_interface::msg::DetectResult::SharedPtr msg);
    void lidar_callback(const vision_interface::msg::RadarWarn::SharedPtr msg);
    void match_callback(const vision_interface::msg::MatchInfo::SharedPtr msg);
    void apply_detect_result(
        const vision_interface::msg::DetectResult& msg,
        bool debug,
        int* resolve_nonzero_out,
        int* red_matched_out,
        int* blue_matched_out,
        float* min_dist_fail_out,
        double* max_time_diff_fail_out);
    std::vector<Kalman_filter_plus> KFs;
    vision_interface::msg::RadarWarn lidar_detect;
    vision_interface::msg::MatchInfo match_info;
    std::mutex latest_detect_mutex_;
    vision_interface::msg::DetectResult latest_detect_msg_;
    bool has_latest_detect_ = false;
    uint64_t detect_callback_count_ = 0;
};
}//namespace tdt_radar

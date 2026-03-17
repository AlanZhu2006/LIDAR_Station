#include "kalman_filter.h"
#include "filter_plus.h"
#include <pcl_conversions/pcl_conversions.h>

namespace tdt_radar{

KalmanFilter::KalmanFilter(const rclcpp::NodeOptions& node_options):rclcpp::Node("kalman_filter_node",node_options)
{
    this->declare_parameter<bool>("debug_camera_match", false);
    this->declare_parameter<double>("camera_detect_radius", 1.0);
    this->declare_parameter<bool>("publish_stationary_targets", false);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_cluster", 10, std::bind(&KalmanFilter::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_kalman", 10);
    radar_pub_ = this->create_publisher<vision_interface::msg::Radar2Sentry>("/radar2sentry", 10);
    radar_detect_pub_ = this->create_publisher<vision_interface::msg::DetectResult>("/kalman_detect", 10);
    sub_detect_= this->create_subscription<vision_interface::msg::DetectResult>("/resolve_result", rclcpp::SensorDataQoS(), std::bind(&KalmanFilter::detect_callback, this, std::placeholders::_1));
    sub_lidar_ = this->create_subscription<vision_interface::msg::RadarWarn>("/lidar_detect", 10, std::bind(&KalmanFilter::lidar_callback, this, std::placeholders::_1));
    sub_match_ = this->create_subscription<vision_interface::msg::MatchInfo>("/match_info", 10, std::bind(&KalmanFilter::match_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Kalman_filter_Node has been started.");
}

void KalmanFilter::match_callback(const vision_interface::msg::MatchInfo::SharedPtr msg)
{
    this->match_info = *msg;
    RCLCPP_INFO(this->get_logger(), "Match_info_callback");
}

void KalmanFilter::detect_callback(const vision_interface::msg::DetectResult::SharedPtr msg)
{
    rclcpp::Time time = msg->header.stamp;
    bool debug = this->get_parameter("debug_camera_match").as_bool();
    float detect_r = static_cast<float>(this->get_parameter("camera_detect_radius").as_double());
    for (auto &kf : KFs) {
        kf.detect_r = detect_r;
    }
    int red_matched = 0, blue_matched = 0;
    float min_dist_fail = 1e9;
    double max_time_diff_fail = 0;

    for(int i=0;i<6;i++)
    {
        pcl::PointXY red_point;
        red_point.x = msg->red_x[i];
        red_point.y = msg->red_y[i];
        if(red_point.x == 0 || red_point.y == 0)continue;
        for(auto &kf : KFs)
        {
            float d; double t;
            if (kf.camera_match(time, red_point, 2, i, debug ? &d : nullptr, debug ? &t : nullptr)) {
                red_matched++;
            } else if (debug && KFs.size() > 0) {
                if (d >= 0 && d < min_dist_fail) min_dist_fail = d;
                if (t > max_time_diff_fail) max_time_diff_fail = t;
            }
        }

        pcl::PointXY blue_point;
        blue_point.x = msg->blue_x[i];
        blue_point.y = msg->blue_y[i];
        if(blue_point.x == 0 || blue_point.y == 0)continue;
        for(auto &kf : KFs)
        {
            float d; double t;
            if (kf.camera_match(time, blue_point, 0, i, debug ? &d : nullptr, debug ? &t : nullptr)) {
                blue_matched++;
            } else if (debug && KFs.size() > 0) {
                if (d >= 0 && d < min_dist_fail) min_dist_fail = d;
                if (t > max_time_diff_fail) max_time_diff_fail = t;
            }
        }
    }

    if (debug && (red_matched > 0 || blue_matched > 0 || min_dist_fail < 1e8 || max_time_diff_fail > 0)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "camera_match: %d red, %d blue matched | KFs=%zu", red_matched, blue_matched, KFs.size());
        if (red_matched == 0 && blue_matched == 0 && KFs.size() > 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "camera_match 无匹配: 最近距离=%.2fm(需<1m), 最大时间差=%.2fs(需<1s). "
                "可能原因: 1)resolve_result与lidar坐标系不一致(检查field_width_m/field_height_m) 2)时间不同步 3)可临时增大camera_detect_radius",
                min_dist_fail < 1e8 ? min_dist_fail : -1.0, max_time_diff_fail);
        }
    }
    // for(auto &kf : KFs)
    // {
    //     for(int i=kf.history.size() - 1; i >= 0; i--)
    //     {
    //         if(Kalman_filter_plus::GetTimeByRosTime(time)-kf.history[i].first > 5)
    //         {
    //             kf.history.erase(kf.history.begin() + i);
    //         }
    //     }
    // }
}

void KalmanFilter::lidar_callback(const vision_interface::msg::RadarWarn::SharedPtr msg)
{
    this->lidar_detect = *msg;
    // RCLCPP_INFO(this->get_logger(), "Lidar_detect_callback");
}

void KalmanFilter::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    rclcpp::Time time = msg->header.stamp;
    bool debug = this->get_parameter("debug_camera_match").as_bool();
    float detect_r = static_cast<float>(this->get_parameter("camera_detect_radius").as_double());
    bool publish_stationary_targets = this->get_parameter("publish_stationary_targets").as_bool();
    auto now_time = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXY>);
    pcl::fromROSMsg(*msg, *cloud);
    for(auto point : cloud->points)
    {
        pcl::PointXY point_xy;
        point_xy.x = point.x;
        point_xy.y = point.y;
        cloud_xy->points.push_back(point_xy);
    }
    if(cloud_xy->points.size() == 0)return;
    for(auto &kf : KFs)
    {
        kf.detect_r = detect_r;
        kf.update_predict_point();
        kf.has_updated = false;
    }

    for(auto point : cloud_xy->points)//对于每个点
    //如果遍历所有卡尔曼都没找到能够匹配的，新建一个卡尔曼
    //若找到了1个，则更新这个卡尔曼
    //若找到了多个，则更新距离最近的那个
    {
        std::vector<int> match_kf_indexs;
        for(int i = 0; i < this->KFs.size(); i++)
        {
            if(KFs[i].match(point)){
                match_kf_indexs.push_back(i);
            }
        }
        if(match_kf_indexs.size() == 0)
        {
            Kalman_filter_plus kf(point, time);
            kf.detect_r = detect_r;
            KFs.push_back(kf);
            // std::cout<<"new kf"<<std::endl;
        }
        else if(match_kf_indexs.size() == 1)
        {
            KFs[match_kf_indexs[0]].update(point, time);
            // std::cout<<"update kf"<<std::endl;
        }
        else
        {
            float min_distance = 1000000;
            int min_index = 0;
            for(auto index : match_kf_indexs)
            {
                float distance = KFs[index].Distance(KFs[index].predict_point, point);
                if(distance < min_distance)
                {
                    min_distance = distance;
                    min_index = index;
                }
            }
            KFs[min_index].update(point, time);
            // std::cout<<"find kf"<<std::endl;
        }
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    int stationary_filtered = 0;
    int published_points = 0;
    for(int i = KFs.size() - 1; i >= 0; i--)
    {
        if((KFs[i].last_time) > 1.5){
            KFs.erase(KFs.begin() + i);
            // std::cout<<"delete kf"<<std::endl;
        }
        else
        {
            bool stationary = KFs[i].is_stationary();
            if (stationary && !publish_stationary_targets) {
                stationary_filtered++;
                continue;  // 静止目标默认不发布
            }
            pcl::PointXYZRGB point;
            pcl::PointXY out_pt = KFs[i].get_output_point();
            point.x = out_pt.x;
            point.y = out_pt.y;
            point.z = 1.5;
            int color = KFs[i].get_color();
            switch (color)
            {
            case 0:
                point.b = 255;
                break;

            case 2:
                point.r = 255;
                break;

            default:
                point.r = KFs[i].color[0];
                point.g = KFs[i].color[1];
                point.b = KFs[i].color[2];
                break;
            }
            cloud_filtered->points.push_back(point);
            published_points++;
        }
    }
    cloud_filtered->header.frame_id = "rm_frame";
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header.frame_id = "rm_frame";
    output.header.stamp = msg->header.stamp;
    pub_->publish(output);
    auto end_time = std::chrono::steady_clock::now();
    float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
    // RCLCPP_INFO(this->get_logger(), "Kalman Callback time is %f ms", dur_time);
    vision_interface::msg::DetectResult detect_msg;
    detect_msg.header.stamp = msg->header.stamp;
    for(auto kf : KFs)
    {
        if(kf.detect_history.size()==0)continue;
        if(kf.is_stationary() && !publish_stationary_targets)continue;
        pcl::PointXY out_pt = kf.get_output_point();
        if(kf.get_color() == 0)//蓝色
        {
            int number= kf.get_number();
            detect_msg.blue_x[number] = out_pt.x;
            detect_msg.blue_y[number] = out_pt.y;
        }
        if(kf.get_color() == 2)//红色
        {
            int number= kf.get_number();
            detect_msg.red_x[number] = out_pt.x;
            detect_msg.red_y[number] = out_pt.y;
        }
    }
    if(match_info.self_color==0){
        for(int i=0;i<6;i++){
            if(detect_msg.blue_x[i]!=0&&detect_msg.blue_y[i]!=0){
                detect_msg.blue_x[i]=28-detect_msg.blue_x[i];
                detect_msg.blue_y[i]=15-detect_msg.blue_y[i];
            }
            if(detect_msg.red_x[i]!=0&&detect_msg.red_y[i]!=0){
                detect_msg.red_x[i]=28-detect_msg.red_x[i];
                detect_msg.red_y[i]=15-detect_msg.red_y[i];
            }
        }
        if(this->lidar_detect.engine_state==1){
            detect_msg.red_x[1]=9.027;
            detect_msg.red_y[1]=10.838;
            //工程赋值
        }
        if(this->lidar_detect.engine_state==2){
            detect_msg.red_x[1]=1.3;
            detect_msg.red_y[1]=3.245;//
            //工程赋值
        }
    }else{
        if(this->lidar_detect.engine_state==1){
            detect_msg.blue_x[1]=28-9.027;
            detect_msg.blue_y[1]=15-10.838;
            //工程赋值
        }
        if(this->lidar_detect.engine_state==2){
            detect_msg.blue_x[1]=28-1.3;
            detect_msg.blue_y[1]=15-3.245;
            //工程赋值
        }
    }
    if (debug && (published_points > 0 || stationary_filtered > 0 || !KFs.empty())) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "kalman_publish: total_kfs=%zu, published=%d, stationary_filtered=%d, publish_stationary_targets=%s",
            KFs.size(), published_points, stationary_filtered, publish_stationary_targets ? "true" : "false");
    }
    radar_detect_pub_->publish(detect_msg);

    vision_interface::msg::Radar2Sentry radar_msg;
    if(match_info.self_color==0){
        for(int i=0;i<6;i++){
            radar_msg.radar_enemy_x[i]=detect_msg.red_x[i];
            radar_msg.radar_enemy_y[i]=detect_msg.red_y[i];
        }
    }else{
        for(int i=0;i<6;i++){
            radar_msg.radar_enemy_x[i]=detect_msg.blue_x[i];
            radar_msg.radar_enemy_y[i]=detect_msg.blue_y[i];
        }
    }

    radar_pub_->publish(radar_msg);
}
}//namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::KalmanFilter)

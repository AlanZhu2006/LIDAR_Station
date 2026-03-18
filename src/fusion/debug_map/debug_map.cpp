#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_interface/msg/detect_result.hpp>
#include <vision_interface/msg/match_info.hpp>
#include <vision_interface/msg/radar2_sentry.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace {

constexpr int kRobotCount = 6;
constexpr std::array<int, kRobotCount> kRobotLabels = {1, 2, 3, 4, 5, 7};
constexpr double kAlignmentMaxRmseM = 0.5;
constexpr double kAlignmentMaxPointErrorM = 1.0;
constexpr double kAlignmentMinInlierRatio = 0.75;
constexpr int kAlignmentMinInlierCount = 3;

int normalize_self_color(int raw_color)
{
    return raw_color == 1 ? 2 : raw_color;
}

int parse_self_color(const std::string & value)
{
    if (!value.empty()) {
        const char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(value[0])));
        if (upper == 'B') {
            return 0;
        }
    }
    return 2;
}

bool has_position(float x, float y)
{
    return x != 0.0F || y != 0.0F;
}

std::string to_absolute_path(const std::string & raw_path)
{
    if (raw_path.empty()) {
        return raw_path;
    }

    const std::filesystem::path path(raw_path);
    if (path.is_absolute()) {
        return path.lexically_normal().string();
    }
    return std::filesystem::absolute(path).lexically_normal().string();
}

cv::Matx33d build_testmap_pixel_to_world_affine(
    int map_width_px,
    int map_height_px,
    double field_width_m,
    double field_height_m)
{
    const double scale_x = field_width_m / static_cast<double>(map_width_px);
    const double scale_y = field_height_m / static_cast<double>(map_height_px);
    return cv::Matx33d(
        scale_x, 0.0, 0.0,
        0.0, -scale_y, field_height_m,
        0.0, 0.0, 1.0);
}

cv::Matx33d build_world_to_topdown_pixel_affine(
    double x_min_m,
    double y_min_m,
    double resolution_m_per_px,
    int height_px)
{
    return cv::Matx33d(
        1.0 / resolution_m_per_px, 0.0, -x_min_m / resolution_m_per_px,
        0.0, -1.0 / resolution_m_per_px,
        static_cast<double>(height_px - 1) + y_min_m / resolution_m_per_px,
        0.0, 0.0, 1.0);
}

cv::Point world_to_pixel(
    const cv::Matx33d & world_to_pixel_affine,
    float world_x_m,
    float world_y_m,
    int image_width,
    int image_height)
{
    const cv::Vec3d pixel_h = world_to_pixel_affine * cv::Vec3d(world_x_m, world_y_m, 1.0);
    const double px = pixel_h[0] / pixel_h[2];
    const double py = pixel_h[1] / pixel_h[2];

    const int x = std::clamp(static_cast<int>(std::lround(px)), 0, image_width - 1);
    const int y = std::clamp(static_cast<int>(std::lround(py)), 0, image_height - 1);
    return cv::Point(x, y);
}

cv::Mat load_optional_image(const std::string & raw_path)
{
    if (raw_path.empty()) {
        return cv::Mat();
    }
    return cv::imread(to_absolute_path(raw_path), cv::IMREAD_COLOR);
}

YAML::Node select_alignment_matrix_node(const YAML::Node & config)
{
    const YAML::Node runtime_transform = config["runtime_transform"];
    if (runtime_transform && runtime_transform["matrix_3x3"]) {
        return runtime_transform["matrix_3x3"];
    }
    return config["similarity_transform"]["matrix_3x3"];
}

bool alignment_metrics_are_acceptable(const YAML::Node & config, std::string & failure_reason)
{
    const YAML::Node error = config["error"];
    if (!error) {
        return true;
    }

    const int num_pairs = error["num_pairs"] ? error["num_pairs"].as<int>() : 0;
    const int num_inliers = error["num_inliers"] ? error["num_inliers"].as<int>() : num_pairs;
    const double rmse_m =
        error["rmse_m"] ? error["rmse_m"].as<double>() : kAlignmentMaxRmseM + 1.0;
    const double max_m =
        error["max_m"] ? error["max_m"].as<double>() : kAlignmentMaxPointErrorM + 1.0;
    const int required_inliers = std::max(
        kAlignmentMinInlierCount,
        static_cast<int>(std::ceil(static_cast<double>(num_pairs) * kAlignmentMinInlierRatio)));

    if (num_pairs < 3) {
        failure_reason = "too few correspondence pairs";
        return false;
    }
    if (num_inliers < required_inliers) {
        failure_reason = "too few inlier correspondences";
        return false;
    }
    if (rmse_m > kAlignmentMaxRmseM) {
        failure_reason = "rmse exceeds runtime threshold";
        return false;
    }
    if (max_m > kAlignmentMaxPointErrorM) {
        failure_reason = "max error exceeds runtime threshold";
        return false;
    }
    return true;
}

}  // namespace

namespace tdt_radar {

class DebugMap : public rclcpp::Node {
public:
    explicit DebugMap(const rclcpp::NodeOptions & options)
        : Node("debug_map", options)
    {
        this->declare_parameter<std::string>(
            "alignment_config_path", "config/local/testmap_to_rm_frame.yaml");
        this->declare_parameter<std::string>("self_color", "R");
        this->declare_parameter<bool>("publish_legacy_radar_topic", true);
        this->declare_parameter<std::string>("legacy_radar_topic", "/Radar2Sentry");

        const char * display_env = std::getenv("DISPLAY");
        show_window_ = display_env != nullptr && display_env[0] != '\0';
        fallback_self_color_ = parse_self_color(
            this->get_parameter("self_color").as_string());

        load_background();

        detect_sub_ = this->create_subscription<vision_interface::msg::DetectResult>(
            "/kalman_detect",
            rclcpp::SensorDataQoS(),
            std::bind(&DebugMap::detect_callback, this, std::placeholders::_1));
        match_info_sub_ = this->create_subscription<vision_interface::msg::MatchInfo>(
            "/match_info",
            10,
            std::bind(&DebugMap::match_info_callback, this, std::placeholders::_1));
        radar_sub_ = this->create_subscription<vision_interface::msg::Radar2Sentry>(
            "/radar2sentry",
            rclcpp::SensorDataQoS(),
            std::bind(&DebugMap::radar_callback, this, std::placeholders::_1));
        debug_map_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/map_2d", 10);

        if (this->get_parameter("publish_legacy_radar_topic").as_bool()) {
            legacy_radar_pub_ =
                this->create_publisher<vision_interface::msg::Radar2Sentry>(
                    this->get_parameter("legacy_radar_topic").as_string(),
                    rclcpp::SensorDataQoS());
        }

        if (!show_window_) {
            RCLCPP_INFO(
                this->get_logger(),
                "DISPLAY is not set; publishing /map_2d without opening an OpenCV window.");
        }
    }

private:
    void match_info_callback(const vision_interface::msg::MatchInfo::SharedPtr msg)
    {
        has_match_info_ = true;
        match_info_self_color_ = normalize_self_color(msg->self_color);
    }

    void radar_callback(const vision_interface::msg::Radar2Sentry::SharedPtr msg)
    {
        if (legacy_radar_pub_) {
            legacy_radar_pub_->publish(*msg);
        }
    }

    int effective_self_color() const
    {
        return has_match_info_ ? match_info_self_color_ : fallback_self_color_;
    }

    void detect_callback(const vision_interface::msg::DetectResult::SharedPtr msg)
    {
        if (background_map_.empty()) {
            return;
        }

        auto canvas = background_map_.clone();
        const int self_color = effective_self_color();
        const bool self_is_blue = self_color == 0;
        const auto & xs = self_is_blue ? msg->red_x : msg->blue_x;
        const auto & ys = self_is_blue ? msg->red_y : msg->blue_y;
        const cv::Scalar enemy_color = self_is_blue ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        const std::string enemy_name = self_is_blue ? "red" : "blue";

        cv::putText(
            canvas,
            "Enemy fused positions on NYUSH map warped into rm_frame",
            cv::Point(16, 28),
            cv::FONT_HERSHEY_SIMPLEX,
            0.65,
            cv::Scalar(240, 240, 240),
            2,
            cv::LINE_AA);
        cv::putText(
            canvas,
            "source=/kalman_detect enemy=" + enemy_name + " self_color=" +
                std::string(self_is_blue ? "B" : "R"),
            cv::Point(16, 56),
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            cv::Scalar(210, 210, 210),
            2,
            cv::LINE_AA);

        for (int i = 0; i < kRobotCount; ++i) {
            if (!has_position(xs[i], ys[i])) {
                continue;
            }

            const cv::Point pixel = world_to_pixel(
                world_to_pixel_affine_,
                xs[i],
                ys[i],
                canvas.cols,
                canvas.rows);
            cv::circle(canvas, pixel, 9, enemy_color, -1, cv::LINE_AA);
            cv::circle(canvas, pixel, 12, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(
                canvas,
                std::to_string(kRobotLabels[i]),
                cv::Point(pixel.x + 10, pixel.y - 10),
                cv::FONT_HERSHEY_SIMPLEX,
                0.65,
                enemy_color,
                2,
                cv::LINE_AA);
        }

        auto image_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", canvas).toImageMsg();
        image_msg->header = msg->header;
        image_msg->header.frame_id = "rm_frame";
        debug_map_pub_->publish(*image_msg);

        if (show_window_) {
            cv::imshow("map", canvas);
            cv::waitKey(1);
        }
    }

    void load_background()
    {
        const std::string config_path = to_absolute_path(
            this->get_parameter("alignment_config_path").as_string());

        try {
            const YAML::Node config = YAML::LoadFile(config_path);
            const YAML::Node testmap = config["testmap"];
            const YAML::Node lidar_topdown = config["lidar_topdown"];
            const YAML::Node matrix_node = select_alignment_matrix_node(config);
            if (!testmap || !lidar_topdown || !matrix_node || !matrix_node.IsSequence() ||
                matrix_node.size() != 3) {
                throw std::runtime_error("alignment YAML is missing required sections");
            }

            const std::string testmap_image_path = testmap["image_path"].as<std::string>();
            const double field_width_m = testmap["field_width_m"].as<double>();
            const double field_height_m = testmap["field_height_m"].as<double>();
            const double x_min_m = lidar_topdown["x_min_m"].as<double>();
            const double y_min_m = lidar_topdown["y_min_m"].as<double>();
            const double resolution_m_per_px =
                lidar_topdown["resolution_m_per_px"].as<double>();
            const int width_px = lidar_topdown["width_px"].as<int>();
            const int height_px = lidar_topdown["height_px"].as<int>();

            cv::Mat topdown_canvas(
                height_px,
                width_px,
                CV_8UC3,
                cv::Scalar(20, 20, 20));

            world_to_pixel_affine_ = build_world_to_topdown_pixel_affine(
                x_min_m, y_min_m, resolution_m_per_px, height_px);

            const cv::Mat lidar_topdown_image = load_optional_image(
                lidar_topdown["image_path"].as<std::string>());
            if (!lidar_topdown_image.empty()) {
                cv::Mat gray_overlay;
                cv::cvtColor(lidar_topdown_image, gray_overlay, cv::COLOR_BGR2GRAY);
                cv::Mat mask;
                cv::threshold(gray_overlay, mask, 1, 255, cv::THRESH_BINARY);
                cv::Mat subtle_overlay(
                    lidar_topdown_image.rows,
                    lidar_topdown_image.cols,
                    CV_8UC3,
                    cv::Scalar(55, 55, 55));
                subtle_overlay.copyTo(topdown_canvas, mask);
            }

            std::string alignment_failure_reason;
            const bool use_testmap_overlay =
                alignment_metrics_are_acceptable(config, alignment_failure_reason);
            if (!use_testmap_overlay) {
                background_map_ = topdown_canvas;
                RCLCPP_WARN(
                    this->get_logger(),
                    "Alignment config %s failed quality checks for debug_map (%s); showing LiDAR topdown only.",
                    config_path.c_str(),
                    alignment_failure_reason.c_str());
                return;
            }

            const cv::Mat runtime_map = load_optional_image(testmap_image_path);
            if (runtime_map.empty()) {
                throw std::runtime_error(
                    "failed to load NYUSH runtime map: " + to_absolute_path(testmap_image_path));
            }

            cv::Matx33d similarity = cv::Matx33d::eye();
            for (int row = 0; row < 3; ++row) {
                const YAML::Node row_node = matrix_node[row];
                if (!row_node.IsSequence() || row_node.size() != 3) {
                    throw std::runtime_error("similarity matrix must be 3x3");
                }
                for (int col = 0; col < 3; ++col) {
                    similarity(row, col) = row_node[col].as<double>();
                }
            }

            const cv::Matx33d testmap_to_world = build_testmap_pixel_to_world_affine(
                runtime_map.cols, runtime_map.rows, field_width_m, field_height_m);
            const cv::Matx33d testmap_to_topdown =
                world_to_pixel_affine_ * similarity * testmap_to_world;

            cv::Mat warped_testmap(
                height_px,
                width_px,
                CV_8UC3,
                cv::Scalar(0, 0, 0));
            cv::warpPerspective(
                runtime_map,
                warped_testmap,
                cv::Mat(testmap_to_topdown),
                warped_testmap.size(),
                cv::INTER_LINEAR,
                cv::BORDER_CONSTANT,
                cv::Scalar(0, 0, 0));

            cv::Mat source_mask(
                runtime_map.rows,
                runtime_map.cols,
                CV_8UC1,
                cv::Scalar(255));
            cv::Mat warped_mask(height_px, width_px, CV_8UC1, cv::Scalar(0));
            cv::warpPerspective(
                source_mask,
                warped_mask,
                cv::Mat(testmap_to_topdown),
                warped_mask.size(),
                cv::INTER_NEAREST,
                cv::BORDER_CONSTANT,
                cv::Scalar(0));
            warped_testmap.copyTo(topdown_canvas, warped_mask);

            background_map_ = topdown_canvas;
            RCLCPP_INFO(
                this->get_logger(),
                "Loaded debug_map background from %s using NYUSH map %s",
                config_path.c_str(),
                to_absolute_path(testmap_image_path).c_str());
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to load aligned NYUSH debug map from %s: %s",
                config_path.c_str(),
                ex.what());
            background_map_ = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(20, 20, 20));
            world_to_pixel_affine_ = cv::Matx33d::eye();
        }
    }

    rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr detect_sub_;
    rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr match_info_sub_;
    rclcpp::Subscription<vision_interface::msg::Radar2Sentry>::SharedPtr radar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_map_pub_;
    rclcpp::Publisher<vision_interface::msg::Radar2Sentry>::SharedPtr legacy_radar_pub_;

    cv::Mat background_map_;
    cv::Matx33d world_to_pixel_affine_ = cv::Matx33d::eye();
    bool show_window_ = false;
    bool has_match_info_ = false;
    int fallback_self_color_ = 2;
    int match_info_self_color_ = 2;
};

}  // namespace tdt_radar

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tdt_radar::DebugMap>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>

#include "MvCameraControl.h"

class HikCameraNode : public rclcpp::Node {
public:
    HikCameraNode() : Node("hik_camera_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Hikvision Camera Node...");

        target_width_ = this->declare_parameter<int>("publish_width", 0);
        target_height_ = this->declare_parameter<int>("publish_height", 0);
        poll_period_ms_ = this->declare_parameter<int>("poll_period_ms", 20);
        if (poll_period_ms_ < 1) {
            poll_period_ms_ = 1;
        }
        grab_timeout_ms_ = this->declare_parameter<int>("grab_timeout_ms", 5);
        if (grab_timeout_ms_ < 0) {
            grab_timeout_ms_ = 0;
        }
        target_fps_ = this->declare_parameter<double>("target_fps", 30.0);
        brightness_gamma_ = this->declare_parameter<double>("brightness_gamma", 0.7);
        brightness_bias_ = this->declare_parameter<int>("brightness_bias", 20);
        buildBrightnessLut();

        const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", image_qos);
        
        if (!initCamera()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera!");
            return;
        }
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(poll_period_ms_),
            std::bind(&HikCameraNode::grabImage, this));
        
        RCLCPP_INFO(this->get_logger(), "Camera initialized successfully!");
    }
    
    ~HikCameraNode() {
        delete[] buffer_;
        if (handle_) {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
        }
    }

private:
    bool initCamera() {
        MV_CC_DEVICE_INFO_LIST device_list;
        memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        
        int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
        if (ret != MV_OK) {
            RCLCPP_ERROR(this->get_logger(), "Enum devices failed! ret = 0x%x", ret);
            return false;
        }
        
        if (device_list.nDeviceNum == 0) {
            RCLCPP_ERROR(this->get_logger(), "No camera found!");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Found %d camera(s)", device_list.nDeviceNum);
        
        ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
        if (ret != MV_OK) {
            RCLCPP_ERROR(this->get_logger(), "Create handle failed! ret = 0x%x", ret);
            return false;
        }
        
        ret = MV_CC_OpenDevice(handle_);
        if (ret != MV_OK) {
            RCLCPP_ERROR(this->get_logger(), "Open device failed! ret = 0x%x", ret);
            return false;
        }
        
        // 设置触发模式为关闭（连续采集）
        ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
        if (ret != MV_OK) {
            RCLCPP_WARN(this->get_logger(), "Set trigger mode failed! ret = 0x%x", ret);
        }

        ret = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
        if (ret == MV_OK) {
            ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", static_cast<float>(target_fps_));
            if (ret == MV_OK) {
                RCLCPP_INFO(this->get_logger(), "AcquisitionFrameRate set to %.1f FPS", target_fps_);
            } else {
                RCLCPP_WARN(this->get_logger(), "Set AcquisitionFrameRate failed! ret = 0x%x", ret);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "AcquisitionFrameRateEnable not supported, using camera default FPS");
        }
        
        // 获取图像参数
        MVCC_INTVALUE width_value, height_value;
        MV_CC_GetIntValue(handle_, "Width", &width_value);
        MV_CC_GetIntValue(handle_, "Height", &height_value);
        width_ = width_value.nCurValue;
        height_ = height_value.nCurValue;
        RCLCPP_INFO(this->get_logger(), "Image size: %d x %d", width_, height_);
        RCLCPP_INFO(
            this->get_logger(),
            "Publish size: %d x %d",
            target_width_ > 0 ? target_width_ : static_cast<int>(width_),
            target_height_ > 0 ? target_height_ : static_cast<int>(height_));
        RCLCPP_INFO(this->get_logger(), "Poll period: %d ms, grab timeout: %d ms", poll_period_ms_, grab_timeout_ms_);
        
        // 分配缓冲区
        buffer_size_ = width_ * height_ * 3;
        buffer_ = new unsigned char[buffer_size_];
        
        // 开始采集
        ret = MV_CC_StartGrabbing(handle_);
        if (ret != MV_OK) {
            RCLCPP_ERROR(this->get_logger(), "Start grabbing failed! ret = 0x%x", ret);
            return false;
        }
        
        return true;
    }

    void buildBrightnessLut() {
        if (std::abs(brightness_gamma_ - 1.0) <= 1e-3 && brightness_bias_ == 0) {
            brightness_lut_.release();
            return;
        }

        brightness_lut_ = cv::Mat(1, 256, CV_8UC1);
        for (int i = 0; i < 256; ++i) {
            float value = static_cast<float>(i);
            if (std::abs(brightness_gamma_ - 1.0) > 1e-3) {
                const float inv_gamma = static_cast<float>(1.0 / brightness_gamma_);
                value = std::pow(value / 255.0f, inv_gamma) * 255.0f;
            }
            value += static_cast<float>(brightness_bias_);
            brightness_lut_.at<unsigned char>(0, i) = static_cast<unsigned char>(
                std::clamp(value, 0.0f, 255.0f));
        }
    }
    
    void grabImage() {
        if (!handle_) return;
        
        MV_FRAME_OUT frame_out;
        memset(&frame_out, 0, sizeof(MV_FRAME_OUT));
        
        int ret = MV_CC_GetImageBuffer(handle_, &frame_out, grab_timeout_ms_);
        if (ret != MV_OK) {
            return;
        }
        
        // 转换为 BGR
        MV_CC_PIXEL_CONVERT_PARAM convert_param;
        memset(&convert_param, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
        convert_param.nWidth = frame_out.stFrameInfo.nWidth;
        convert_param.nHeight = frame_out.stFrameInfo.nHeight;
        convert_param.pSrcData = frame_out.pBufAddr;
        convert_param.nSrcDataLen = frame_out.stFrameInfo.nFrameLen;
        convert_param.enSrcPixelType = frame_out.stFrameInfo.enPixelType;
        convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        convert_param.pDstBuffer = buffer_;
        convert_param.nDstBufferSize = buffer_size_;
        
        ret = MV_CC_ConvertPixelType(handle_, &convert_param);
        if (ret != MV_OK) {
            MV_CC_FreeImageBuffer(handle_, &frame_out);
            return;
        }
        
        // 创建 OpenCV Mat
        cv::Mat img(convert_param.nHeight, convert_param.nWidth, CV_8UC3, buffer_);

        int publish_width = static_cast<int>(convert_param.nWidth);
        int publish_height = static_cast<int>(convert_param.nHeight);
        if (target_width_ > 0 && target_height_ > 0) {
            publish_width = target_width_;
            publish_height = target_height_;
        } else if (target_width_ > 0) {
            publish_width = target_width_;
            publish_height = static_cast<int>(
                std::round(static_cast<double>(convert_param.nHeight) * publish_width / static_cast<double>(convert_param.nWidth)));
        } else if (target_height_ > 0) {
            publish_height = target_height_;
            publish_width = static_cast<int>(
                std::round(static_cast<double>(convert_param.nWidth) * publish_height / static_cast<double>(convert_param.nHeight)));
        }

        cv::Mat publish_img;
        if (publish_width == img.cols && publish_height == img.rows) {
            publish_img = img;
        } else {
            cv::resize(img, publish_img, cv::Size(publish_width, publish_height), 0, 0, cv::INTER_LINEAR);
        }

        // 亮度调整改成 LUT，避免每帧整图浮点 pow 带来额外延迟
        if (!brightness_lut_.empty()) {
            cv::Mat adjusted_img;
            cv::LUT(publish_img, brightness_lut_, adjusted_img);
            publish_img = adjusted_img;
        }
        
        // 发布图像
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", publish_img).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";
        image_pub_->publish(*msg);
        
        MV_CC_FreeImageBuffer(handle_, &frame_out);
    }
    
    void* handle_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    unsigned char* buffer_ = nullptr;
    unsigned int buffer_size_ = 0;
    unsigned int width_ = 0;
    unsigned int height_ = 0;
    int target_width_ = 0;
    int target_height_ = 0;
    int poll_period_ms_ = 20;
    int grab_timeout_ms_ = 5;
    double target_fps_ = 30.0;
    double brightness_gamma_ = 0.7;
    int brightness_bias_ = 20;
    cv::Mat brightness_lut_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HikCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

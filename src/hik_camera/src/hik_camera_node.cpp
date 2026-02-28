#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "MvCameraControl.h"

class HikCameraNode : public rclcpp::Node {
public:
    HikCameraNode() : Node("hik_camera_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Hikvision Camera Node...");
        
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", 10);
        
        if (!initCamera()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera!");
            return;
        }
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 FPS
            std::bind(&HikCameraNode::grabImage, this));
        
        RCLCPP_INFO(this->get_logger(), "Camera initialized successfully!");
    }
    
    ~HikCameraNode() {
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
        
        // 获取图像参数
        MVCC_INTVALUE width_value, height_value;
        MV_CC_GetIntValue(handle_, "Width", &width_value);
        MV_CC_GetIntValue(handle_, "Height", &height_value);
        width_ = width_value.nCurValue;
        height_ = height_value.nCurValue;
        RCLCPP_INFO(this->get_logger(), "Image size: %d x %d", width_, height_);
        
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
    
    void grabImage() {
        if (!handle_) return;
        
        MV_FRAME_OUT frame_out;
        memset(&frame_out, 0, sizeof(MV_FRAME_OUT));
        
        int ret = MV_CC_GetImageBuffer(handle_, &frame_out, 100);
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
        
        // Resize 图像到模型训练分辨率 (4096x3000)
        // 这是因为 YOLO 模型是在 4096x3000 分辨率图像上训练的
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(4096, 3000), 0, 0, cv::INTER_LINEAR);
        
        // 发布图像
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized_img).toImageMsg();
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
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HikCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

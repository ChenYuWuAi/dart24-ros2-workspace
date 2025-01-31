#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <string>
#include <cstdio>
#include <ctime>
#include <sstream>
#include <unistd.h>
#include <unordered_map>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "camera_hal/camera_driver.hpp"
#include "camera_hal/camera_dh.hpp"
#include "camera_hal/camera_v4l2.hpp"

using namespace std::chrono_literals;
using namespace CameraHAL;

class NodeCamera : public rclcpp_lifecycle::LifecycleNode
{
private:
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr publisher_test_;
    rclcpp::TimerBase::SharedPtr timer_;

    CameraDriver *camera_;
    int exposure_time;

    std::shared_ptr<std::thread> capture_thread_; // 采集线程

    bool running;

    void initCamera(int exposure_time) // 1-3ms 0=>auto
    {
        // do sth to make sure you open the right camera
        // camera_=new VideoCapture;
        RCLCPP_INFO(this->get_logger(), "Initializing camera...");
        // if (!camera_.open(0, 2))
        // {
        //     RCLCPP_ERROR(this->get_logger(), "!!!!!!!!!!!!!!!!!!!!!no camera!!!!!!!!!!!!!!!!!!!!!!!!!");
        //     rclcpp::shutdown();
        // }
        // camera_->setExposureTime(exposure_time);
        // camera_->setVideoFormat(1280, 1024, true);
        // camera_->setFPS(30.0);
        // camera_->setBalanceRatio(1.6, 1.3, 2.0, true);
        // camera_->setGain(1);
        // camera_->setGamma(1);

        // camera_->startStream();
        // camera_->closeStream();

        // camera_->startStream();
        RCLCPP_INFO(this->get_logger(), "Camera initialized.");
    }

    void camera_capture_thread()
    {
        // 从相机读取图像
        int width = this->get_parameter("image_width").as_int();
        int height = this->get_parameter("image_height").as_int();
        cv::Mat image(height, width, CV_8UC2);
        RCLCPP_INFO(this->get_logger(), "Start capturing...");
        std::stringstream ss;
        auto now = std::time(nullptr);
        ss << "camera." << std::put_time(std::localtime(&now), "%y%m%d%H%M%S") << ".avi";
        char cwd[256];
        getcwd(cwd, sizeof(cwd));
        RCLCPP_INFO_STREAM(this->get_logger(), "Recording to " << cwd << "/" << ss.str());
        // 录像到当前工作目录，文件名为camera.xxxxxxxxxx(yymmddhhmmss)，格式为avi，6fps
        cv::VideoWriter video(ss.str(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, cv::Size(width, height));

        while (running && rclcpp::ok())
        {
            camera_->read(image);

            // 每3次发布图像 帧率10fps
            static int count = 0;
            static auto last_pub = std::chrono::system_clock::now();
            // 计算实际帧率
            auto now = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pub);
            last_pub = now;

            if (count++ % 10 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Actual fps: %f", 1000.0 / duration.count());
                video.write(image);
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
                publisher_test_->publish(*msg);
                RCLCPP_INFO(this->get_logger(), "Image published.");
            }
            std::this_thread::sleep_for(16ms); // 60fps
        }
        video.release();
        RCLCPP_INFO(this->get_logger(), "Capture thread stopped.");
    }

public:
    explicit NodeCamera(rclcpp::NodeOptions options)
        : rclcpp_lifecycle::LifecycleNode("camera",
                                          options)
    {
        exposure_time = 0;
        RCLCPP_INFO(this->get_logger(), "Camera_Node created.");
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &pre_state)
    {
        LifecycleNode::on_configure(pre_state);

        // 初始化相机
        publisher_test_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);

        // 从本节点参数（预加载了camera_<device>_params.yaml）中读取各项初始化参数
        if (!this->has_parameter("camera_type"))
        {
            RCLCPP_ERROR(this->get_logger(), "No camera type specified. Please set the camera_type parameter.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        auto params = this->get_parameter("camera_type");
        std::string camera_type = params.as_string();

        // 根据camera_type选择相机驱动
        if (camera_type == "v4l2")
            camera_ = new CameraDriver_V4L2;
        else if (camera_type == "dh")
            camera_ = new CameraDriver_DH;
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown camera type: %s", camera_type.c_str());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "Camera type: %s", camera_type.c_str());

        // 读取相机参数
        auto params_map = this->get_parameter("camera_params").as_string_array();
        std::unordered_map<std::string, std::string> camera_params;
        for (auto &param : params_map)
        {
            std::string key = param.substr(0, param.find('='));
            std::string value = param.substr(param.find('=') + 1);
            RCLCPP_INFO(this->get_logger(), "Camera param: %s: %s", key.c_str(), value.c_str());
            camera_params[key] = value;
        }

        if (!camera_->open(camera_params))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "Camera opened.");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &pre_state)
    {
        LifecycleNode::on_activate(pre_state);
        running = true;
        if (capture_thread_ == nullptr)
        {
            capture_thread_ = std::make_shared<std::thread>(std::bind(&NodeCamera::camera_capture_thread, this));
            RCLCPP_INFO(this->get_logger(), "Capture thread started.");
            capture_thread_->detach();
        }
        else
            RCLCPP_INFO(this->get_logger(), "Capture thread already running.");

        RCLCPP_INFO(this->get_logger(),
                    "Camera_Node on_activate is called, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &pre_state)
    {
        LifecycleNode::on_deactivate(pre_state);

        running = false;
        RCLCPP_INFO(this->get_logger(), "Capture thread stopping...");
        capture_thread_->join();

        RCLCPP_INFO(this->get_logger(),
                    "Camera_Noder on_deactivate is called, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &pre_state)
    {
        // 释放资源，清空智能指针
        publisher_test_.reset();
        timer_.reset();
        running = false;
        if (capture_thread_ != nullptr)
        {
            capture_thread_->join();
            capture_thread_.reset();
        }

        camera_ = 0;
        RCLCPP_INFO(this->get_logger(),
                    "LifecycleTalker on_cleanup is called, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &pre_state)
    {
        // 释放资源，清空智能指针
        publisher_test_.reset();
        timer_.reset();
        camera_ = 0;
        RCLCPP_INFO(this->get_logger(),
                    "LifecycleTalker on_shutdown is called, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().use_intra_process_comms(false);
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<NodeCamera>(options);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
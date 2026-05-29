#ifndef CORE_NODE_HPP
#define CORE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <memory>
#include <string>
#include <vector>

#include "prepare_algorithm.h"
#include "tf.hpp"
#include "ekf.hpp"
#include "plotter.hpp"
#include "serial_driver_interfaces/msg/serial_driver.hpp"

class CoreNode: public rclcpp::Node
{
public:

    // ==================== 构造和析构函数 ====================
    
    CoreNode();
    ~CoreNode();

private:

    // ==================== 初始化分离模块 ====================

    void InitParams(); // 负责声明和获取 ROS2 参数
    void InitROS2();   // 负责初始化 Pub/Sub 和 定时器/线程

    // ==================== 核心逻辑流 ====================

    void VideoReading(); // 本地视频读取专用线程函数
    void CameraImageCallback(const sensor_msgs::msg::Image::SharedPtr msg); // ROS2 相机节点订阅回调函数
    void CoreLogic(cv::Mat& frame, rclcpp::Time current_image_time); // 核心逻辑函数，接收图像和时间戳，执行预处理、pnp、ekf、发布等全部核心功能

    // ==================== 工具函数 ====================
    
    void ShowImg(); // 显示当前 img_show 的函数
    void UpdatePlotter(Eigen::Vector3d armorplate_center_now, 
                       Eigen::Vector3d armorplate_center_filter, 
                       Eigen::Vector3d armorplate_center_predict); // 更新示波器的函数，接收实时点、滤波点、预测点

    // ==================== ROS2相关 ====================

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rcl_interfaces::msg::SetParametersResult OnParameterChange(const std::vector<rclcpp::Parameter>& params); // 参数动态修改回调函数，接收修改后的参数列表

    rclcpp::Publisher<serial_driver_interfaces::msg::SerialDriver>::SharedPtr serial_pub_; // 串口消息发布器
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_; // 参数动态修改回调函数的句柄，用于在析构函数中取消注册
    
    // ==================== 多线程 ====================

    std::thread video_thread_; // 本地视频读取线程

    // ==================== 核心算法类对象 ====================

    std::unique_ptr<TF> tf_;
    std::unique_ptr<EKF> ekf_;
    std::unique_ptr<Plotter> plotter_;
    std::unique_ptr<Prepare> prepare_;       // prepare 类对象，包含预处理函数、寻找灯条函数、配对函数等

    std::vector<Strip> strips_;               // 灯条类集合，从 prepare.findAndJudgeLightStrip() 返回
    std::vector<ArmorPlate> armorplates_;     // 装甲板集合，从 prepare.pairStrip() 返回

    // ==================== 图像与时间缓存 ====================

    cv::Mat img_;                   // 原图
    cv::Mat img_show_;              // 复制一份用来显示信息的图
    rclcpp::Time last_image_time_;  // 上一次收到图像的时间戳

    // ==================== EKF 运行状态机变量 ====================

    int continuous_count_ = 0;      // 记录连续有效的帧数，刚收到数据的前几帧不要（因为不稳定）
    int lost_count_ = 0;            // 记录连续丢失的帧数，丢失一定次数后不给ekf了，并且重置
    int max_lost_count_ = 60;       // 最大连续丢失量，超过这个就不用 ekf 的预测了，大概 60 / 200 = 0.3s
    bool ekf_ready_ = false;        // ekf 是否已经稳定跟踪
    int tracking_id_ = 3;           // 当前正在追踪的最好的装甲板 ID

    // ============ 配置参数 (Config) 从参数服务器获取，可修改 ============

    bool show_logger_about_time_;   // 是否显示 core 节点中的每帧耗时日志（计算耗时）
    bool show_logger_about_else_;   // 是否显示 core 节点中的其他日志（除计算耗时以外的日志）
    bool show_image_;               // 是否显示 img_show 窗口
    bool show_logger_prepare_;      // 是否显示 prepare 中的日志（与配对相关）
    bool show_logger_ekf_debug_;    // 是否显示 ekf 打印的调试日志
    bool show_plot_;                // 是否画波形图

    bool is_standalone_mode_;       // 单机模式 / 联调模式的切换，决定了父坐标系是谁（主要是为了调试时不依赖电控的 TF 发布）
                                    // true 就是单机模式，父坐标系是 camera_frame；false 就是联调模式，父坐标系是 world
    bool is_video_mode_;            // 是否读取本地视频模式。如果是，就从 VIDEO_PATH 读视频，否则就订阅话题收图
    
    std::string chosen_color_;      // 选择检测的颜色 red / blue
    std::string camera_name_;       // 选择相机名称 mind_vision / galaxy ，会对应不同的 qos
    std::string armorplate_type_;   // 选择装甲板类型 normal / hero ，决定了配对的参数
    std::string video_path_;        // 本地视频路径，只有 IS_VIDEO_MODE = true 才有效
    
    double ekf_predict_time_;       // ekf 预测时间
};

#endif // CORE_NODE_HPP
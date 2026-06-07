#ifndef CORE_NODE_HPP
#define CORE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "tf.hpp"
#include "ekf.hpp"
#include "plotter.hpp"
#include "runge_kutta.hpp"
#include "serial_driver_interfaces/msg/serial_driver.hpp"

#include "yolo_detector.hpp" // yolo 检测器类
#include "yolo_armor.hpp"   // yolo 检测结果的装甲板类

#include "tools/math_tools.hpp"


// ==================== [新增] 追踪器状态机枚举 ====================
// 四阶段逻辑
enum class TrackerState 
{
    LOST,       // 彻底丢失状态：没有目标，滤波器重置
    DETECTING,  // 防抖确认状态：刚看到目标，但还不稳定，不立刻开始跟踪
    TRACKING,   // 稳定跟踪状态：目标连续可见，EKF 正常工作
    TEMP_LOST   // 短暂丢失状态：目标突然消失（被遮挡或小陀螺），EKF 开启盲推外推！
};


// ============================== 目标颜色映射表 ==============================
inline const std::unordered_map<std::string, int> COLOR_MAP = {
    {"blue",   0},
    {"red",    1},
    {"gray",   2},
    {"purple", 3}
};


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
    
    // 【新增】：状态机大脑决策（只改变状态，不执行动作）
    void UpdateTrackerState(bool is_found); 

    // 【新增】：追踪器肢体执行（根据状态去执行 EKF 动作）
    void ExecuteTracker(double dt, rclcpp::Time current_image_time,
                        double& yaw_armorplate_now,
                        Eigen::Vector3d& armorplate_center_now, 
                        Eigen::Vector3d& armorplate_center_filter, 
                        Eigen::Vector3d& armorplate_center_predict, 
                        Eigen::Vector3d& car_center_predict);

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

    // 传统视觉算法相关
    // std::unique_ptr<Prepare> prepare_;       // prepare 类对象，包含预处理函数、寻找灯条函数、配对函数等
    // std::vector<Strip> strips_;               // 灯条类集合，从 prepare.findAndJudgeLightStrip() 返回
    // std::vector<ArmorPlate> armorplates_;     // 装甲板集合，从 prepare.pairStrip() 返回

    // yolo 相关
    std::vector<YoloArmor> yolo_armors_;     // yolo 检测到的装甲板集合，从 yolo_detector_.Detect() 返回

    // ==================== 图像、相机相关与时间缓存 ====================

    cv::Mat img_show_;              // 复制一份用来显示信息的图
    rclcpp::Time last_image_time_;  // 上一次收到图像的时间戳

    cv::Mat K_; // 相机内参矩阵
    cv::Mat D_; // 相机畸变系数

    // ==================== 神经网络 ====================

    std::unique_ptr<YoloDetector> yolo_detector_; // 声明 yolo检测器 指针

    // ==================== EKF 运行状态机变量 ====================

    TrackerState tracker_state_ = TrackerState::LOST; // 初始状态设为彻底丢失

    int detect_count_ = 0;          // 刚发现目标时的防抖计数器
    int temp_lost_count_ = 0;       // 丢失目标时的盲推计数器

    int min_detect_frames_ = 3;     // 最小开始跟踪阈值：连续看到 3 帧才认为是真正发现了目标 (防抖)
    int max_lost_frames_ = 80;      // 最大连续丢失阈值：连续丢失 80 帧 (约0.4s) 才认为是彻底丢失，放弃盲推

    int reject_count_ = 0;          // ekf连续拒绝yolo计数器
    int max_reject_frames_ = 4;     // 最大拒绝yolo阈值：看到了目标又全都没有有效更新连续那么多帧，直接 LOST，不能一直盲推

    int armor_num_ = 4;             // 兵种装甲板数量 (平衡步兵2，前哨站3，其他4)
    
    bool ekf_ready_ = false;        // ekf 是否已经稳定跟踪（初始化ok后，连续跟踪 min_detect_frames_ 帧）

    // 【极其关键】：初始化必须为 0！
    // 这样当相机第一次看到目标时，EKF 就会把该目标认作 0 号板 (车头)。
    // 从而自动建立 "初始视线 = 整车 0 度角" 的相对坐标系！
    int tracking_id_ = 0;           // 当前正在追踪的装甲板 ID

    // ============ 弹道模型 ============

    std::unique_ptr<RungeKutta> ballistic_solver_;

    // ============ 配置参数 (Config) 从参数服务器获取，可修改 ============

    bool show_logger_about_time_;   // 是否显示 core 节点中的每帧耗时日志（计算耗时）
    bool show_logger_about_else_;   // 是否显示 core 节点中的其他日志（除计算耗时以外的日志）
    bool show_logger_pnp_;          // 是否显示 PNP 运行日志
    bool show_image_;               // 是否显示 img_show 窗口
    bool show_plot_;                // 是否画波形图

    bool is_standalone_mode_;       // 单机模式 / 联调模式的切换，决定了父坐标系是谁（主要是为了调试时不依赖电控的 TF 发布）
                                    // true 就是单机模式，父坐标系是 camera_frame；false 就是联调模式，父坐标系是 world
    bool is_video_mode_;            // 是否读取本地视频模式。如果是，就从 VIDEO_PATH 读视频，否则就订阅话题收图
    
    std::string chosen_color_;      // 选择检测的颜色 red / blue
    std::string camera_name_;       // 选择相机名称 mind_vision / galaxy ，会对应不同的 qos
    std::string video_path_;        // 本地视频路径，只有 IS_VIDEO_MODE = true 才有效
    
    double ekf_predict_time_;       // ekf 预测时间 
                                    // todo: 在 corenode 里动态计算！
};

#endif // CORE_NODE_HPP
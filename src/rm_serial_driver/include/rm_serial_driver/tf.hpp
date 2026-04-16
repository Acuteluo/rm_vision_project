# pragma once

#ifndef RM_SERIAL_DRIVER__TF_HPP_
#define RM_SERIAL_DRIVER__TF_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/static_transform_broadcaster.h" // 静态变换广播器
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "serial_driver_interfaces/msg/serial_driver.hpp"
#include "serial_driver_interfaces/msg/send_pnp_info.hpp"
#include<Eigen/Dense>
#include<Eigen/QR>
#include<Eigen/Core>
#include<Eigen/LU>

#include <fstream>
#include <string>

#include <rm_serial_driver/kf_position.hpp>
#include <rm_serial_driver/kf_data.hpp>

// 同一个命名空间
namespace rm_serial_driver
{

/**
 * @brief TF 类，封装所有 TF 广播、监听、查询功能。
 *        需要传入 Node 指针以创建 ROS2 相关的对象（Broadcaster, Buffer 等）。
 */
class TF
{
public:
    // 构造函数拿到 Node 指针以创建 ROS2 相关的对象
    explicit TF(rclcpp::Node* node); 


    // ---------- 更新动态 TF ----------
    /**
     * @brief 根据芯片欧拉角更新 world_frame -> chip_frame 的变换。
     * @param roll 翻滚角（度）
     * @param pitch 俯仰角（度）
     * @param yaw 航向角（度）
     */
    void updateWorldToChip(double roll, double pitch, double yaw);


    // ---------- 查询 TF ----------
    /**
     * @brief 查询【世界坐标系】-> 【装甲板坐标系】和【世界坐标系】->【相机坐标系】是否可以变换
              可以就变换就先滤波，通过引用回传滤波后的最终结果（加上电控的欧拉角），返回1或者0表示是否有效
     * @param pitch
     * @param yaw
     * @param euler_pitch
     * @param euler_yaw
     * @return 1 可变换，0 变换失败（TF 树不完整）
     */
    bool getTransform(float& pitch, float& yaw, float euler_pitch, float euler_yaw);



    /**
     * @brief 直接发布 camera_frame -> armorplate_frame 的变换
     * @param msg& pnp 消息的引用
     */
    void updateCameraToArmorplate(const serial_driver_interfaces::msg::SendPNPInfo& msg);


private:

    // ---------- 静态变换 ----------
    /**
     * @brief 启动静态变换发布，发布 chip_frame -> camera_frame 的变换
     */
    void publishStaticCameraTransform();  // 实际发布静态变换的回调


    rclcpp::Node* node_;  // 拿到 ros2 的节点指针

    // KF 类对象指针
    std::unique_ptr<KalmanFilter> kf_position_;
    std::unique_ptr<KF> kf_data_;

    // 上一次查询的的时间戳
    double last_lookup_time_ = -1;

    // 计数器，这样可以让滤波器用前几帧数据来初始化
    int count = 0;

    // TF 广播器、缓存、监听器
    std::unique_ptr<tf2_ros::TransformBroadcaster> chip_broadcaster_;     // 发布 chip_frame 相关 TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> armorplate_broadcaster_;    // 发布 armorplate_frame TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                 // TF 缓存
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;    // TF 监听器

    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_; // 发布静态变换的广播器

    bool SHOW_LOGGER_ERROR; // 从日志文件读取，是否展示tf查询错误
    bool SHOW_RESULT; // 从日志文件读取，是否展示最终查询结果
};

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__TF_MANAGER_HPP_
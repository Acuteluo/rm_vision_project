# pragma once

#include "src/img_processing/include/strip.h"

#ifndef RM_SERIAL_DRIVER__TF_MANAGER_HPP_
#define RM_SERIAL_DRIVER__TF_MANAGER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// 同一个命名空间
namespace rm_serial_driver
{

/**
 * @brief TF 管理类，封装所有 TF 广播、监听、查询功能。
 *        需要传入 Node 指针以创建 ROS2 相关的对象（Broadcaster, Buffer 等）。
 */
class TF
{
public:
    // 构造函数拿到 Node 指针以创建 ROS2 相关的对象
    explicit TF(rclcpp::Node::SharedPtr node); 


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
     * @brief 先查询是否可以变换，假如可以，则转换+滤波，通过引用让其得到最终结果
     * @return 1 可变换，0 变换失败（TF 树不完整）
     */
    bool getTransform(float& yaw, float& pitch);

    // ---------- 静态变换 ----------
    /**
     * @brief 启动周期性发布 chip_frame -> camera_frame 的静态变换
     * @param period_ms 发布周期（毫秒）
     */
    void startStaticTransformTimer(int period_ms);


    /**
     * @brief 直接发布 camera_frame -> armorplate_frame 的变换
     * @param msg& pnp 消息的引用
     */
    void updateCameraToArmorplate(const serial_driver_interfaces::msg::SendPNPInfo& msg);


private:

    void publishStaticCameraTransform();  // 实际发布静态变换的回调

    rclcpp::Node::SharedPtr node_;  // 拿到 ros2 的节点指针

    // TF 广播器、缓存、监听器
    std::unique_ptr<tf2_ros::TransformBroadcaster> chip_broadcaster_;     // 发布 chip_frame 相关 TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> armorplate_broadcaster_;    // 发布 armorplate_frame TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                 // TF 缓存
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;    // TF 监听器
    rclcpp::TimerBase::SharedPtr static_timer_;   // 静态变换定时器
};

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__TF_MANAGER_HPP_
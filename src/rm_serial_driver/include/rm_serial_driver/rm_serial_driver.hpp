// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.
#pragma once

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rm_serial_driver/packet.hpp" // 需要使用 ReceivePacket 和 SendPacket 结构体

#include "tf.hpp" // tf 类，封装所有 TF 广播、监听、查询功能

#include <mutex> // 锁

#include "statistics_msgs/msg/metrics_message.hpp" // 监听话题发送频率
#include <chrono>

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:

    explicit RMSerialDriver(const rclcpp::NodeOptions &options);

    ~RMSerialDriver() override;

    // 一些用来对比的东西
    float last_tvec[3] = {-9999.99, -9999.99, -9999.99}; // 上一次发送的 t 矩阵数据
    int pnp_same_t_count = 0; // PNP 消息收到重复 t 矩阵的计数器
    double last_pitch = -9999.99; // 上一次发送的 pitch 数据
    double last_yaw = -9999.99; // 上一次发送的 yaw 数据
    int data_same_count = 0; // 最终要发送的数据 重复计数器
    rclcpp::Time send_once_start; // 记录发送前的时刻，用于计算 整次 send_interval 的时间间隔

    int receive_data_count = 0; // 接收数据计数器

    rclcpp::Time last_print; // 统计电控发来消息的频率

private:

    // 接收电控回传数据，并且调用 TF 类方法，更新【世界坐标系】->【芯片坐标系】变换的函数
    void receiveData();

    // 最终发送给串口的函数（已经确认过）
    void ultimateSendData(float pitch, float yaw);

    // 重试打开串口的函数（在接收数据时发生异常时调用）
    void reopenPort();

    // 读取 config 文件中的参数
    void getParams();

    // 订阅 PnP 的回调信息，更新【芯片坐标系】->【装甲板坐标系】的变换
    void PNPCallback(const serial_driver_interfaces::msg::SendPNPInfo msg);

    // 确定两个坐标系是否都已经更新，尝试查询 TF 变换，得到最终数据，并发送串口
    void confirmIfCanSendData();

    // Serial port
    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    // 收数据
    rclcpp::Subscription<serial_driver_interfaces::msg::SendPNPInfo>::SharedPtr pnp_sub_; // 订阅 PnP 话题
    std::thread receive_thread_;

    // 判断坐标系是否更新
    bool world_to_chip_updated_ = false;      // 收到的数据，【世界坐标系】->【芯片坐标系】
    bool chip_to_armorplate_updated_ = false; // 订阅的 PnP 数据，【芯片坐标系】->【装甲板坐标系】

    // 相关类对象指针
    std::unique_ptr<TF> tf;
    // std::unique_ptr<AngleFilter> angle_filter_;

    // 日志相关
    bool SHOW_LOGGER_PNP; // pnp 相关日志
    bool SHOW_LOGGER_RECEIVE; // 收到电控数据相关日志
    bool SHOW_LOGGER_TRY_AND_SEND; // 尝试发送数据相关日志

    mutable std::mutex state_mutex_;   // 加锁。保护共享状态（标志位、重复检测变量等）
};  

} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

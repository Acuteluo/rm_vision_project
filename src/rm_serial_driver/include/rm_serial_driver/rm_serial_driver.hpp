// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.
#pragma once

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <iostream>

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
#include "serial_driver_interfaces/msg/serial_driver.hpp" // 串口消息 发送方 的 话题类型 为 [serial_driver] 类

#include <mutex> // 锁

#include <tf2/LinearMath/Quaternion.h>

#include <fstream>
#include <string>


namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:

    explicit RMSerialDriver(const rclcpp::NodeOptions &options);

    ~RMSerialDriver() override;

    // 一些用来对比是否重复的东西
    double last_send_pitch = -9999.99; // 上一次发送的 pitch 数据
    double last_send_yaw = -9999.99; // 上一次发送的 yaw 数据
    int send_data_same_count = 0; // 最终要发送的数据 重复计数

    rclcpp::Time last_receive_time; // 用来统计电控发来消息的频率

private:

    // 接收电控回传数据，并且调用 TF 类方法，更新【世界坐标系】->【芯片坐标系】变换的函数
    void receiveData();

    // 订阅 最终数据 的回调信息，检查数据是否有效
    void CheckData(const serial_driver_interfaces::msg::SerialDriver msg);

    // 发送给串口数据的函数（检查过有效性的）
    void SendData(float pitch, float yaw);

    // 重试打开串口的函数（在接收数据时发生异常时调用）
    void reopenPort();

    // 读取 config 文件中的参数
    void getParams();

    // 01【世界坐标系】->【芯片坐标系】当前芯片位姿的坐标系 -> 动态，用 R 矩阵
    void updateWorldToChip(double roll, double pitch, double yaw);

    // Serial port
    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    // 收数据
    rclcpp::Subscription<serial_driver_interfaces::msg::SerialDriver>::SharedPtr data_sub_; // 订阅 /serial_driver 话题
    std::thread receive_thread_;


    // 最新收到 电控数据的时间戳
    rclcpp::Time receive_time;
    
    // TF 广播器、缓存、监听器
    std::unique_ptr<tf2_ros::TransformBroadcaster> chip_broadcaster_;


    // 日志相关
    bool SHOW_LOGGER_RECEIVE; // 收到电控数据相关日志
    bool SHOW_LOGGER_TRY_AND_SEND; // 尝试发送数据相关日志

    mutable std::mutex state_mutex_;   // 加锁。保护共享状态（标志位、重复检测变量等）
};  

} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

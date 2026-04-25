# pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/static_transform_broadcaster.h" // 静态变换广播器
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include<Eigen/Dense>
#include<Eigen/QR>
#include<Eigen/Core>
#include<Eigen/LU>

#include <fstream>
#include <string>


/**
 * @brief TF 类，封装所有 TF 广播、监听、查询功能。
 *        需要传入 Node 指针以创建 ROS2 相关的对象（Broadcaster, Buffer 等）。
 */
class TF
{
public:
    // 构造函数拿到 Node 指针以创建 ROS2 相关的对象
    explicit TF(rclcpp::Node* node); 


    // 从参数服务器更新内部开关（运行时调用）
    void updateParamsFromServer();


    // ---------- 查询 TF ----------
    /**
     * @brief 查询【父坐标系】->【装甲板坐标系】是否可以变换
     *        单机模式时父坐标系是 camera_frame，联调模式时父坐标系是 world_frame
              通过引用回传滤波后的最终结果，返回1或者0表示是否有效
     * @param 
     * @param 
     * @return 1 可变换，0 变换失败（TF 树不完整）
     */
    bool getFatherToArmorplateTransform(Eigen::Vector3d& armorplate_center, double& yaw_armor);


    // 查询【世界坐标系】->【相机坐标系】是否可以变换
    bool getWorldToCameraTransform(tf2::Transform& T_world_cam);


    /**
     * @brief 直接发布 camera_frame -> armorplate_frame 的变换
     * @param 
     */
    void updateCameraToArmorplate(Eigen::Matrix3d R, Eigen::Vector3d t);


private:

    // ---------- 静态变换 ----------
    /**
     * @brief 启动静态变换发布，发布 chip_frame -> camera_frame 的变换
     */
    void publishStaticCameraTransform();  // 实际发布静态变换的回调



    rclcpp::Node* node_;  // 拿到 ros2 的节点指针

    // // KF 类对象指针
    // std::unique_ptr<KalmanFilter> kf_position_;
    // std::unique_ptr<KF> kf_data_;


    // TF 广播器、缓存、监听器
    std::unique_ptr<tf2_ros::TransformBroadcaster> armorplate_broadcaster_;    // 发布 armorplate_frame TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                 // TF 缓存
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;    // TF 监听器

    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_; // 发布静态变换的广播器

    bool SHOW_LOGGER_ERROR; // 从日志文件读取，是否展示tf查询错误
    bool SHOW_RESULT; // 从日志文件读取，是否展示最终查询结果

    std::string father_frame; // 从参数服务器读取，查询父坐标系->装甲板坐标系时候要用。单机模式时是 camera_frame，联调模式时是 world_frame
};


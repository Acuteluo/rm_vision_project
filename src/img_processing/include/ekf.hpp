/*
    EKF 

        思路：目前先固定R，根据R发布动态变换（传一次观测数据就发布一次），发布从中心到四周装甲板的变换（十字架模型）
        
        注意我们忽略了装甲板数据的 pitch & roll，意味着所有的都是只有 yaw，也就是底盘一定垂直于地面喵

             传入观测数据 -> 推算几何中心位姿 -> ekf滤波 -> 发布整车中心与四个装甲板的坐标系 -> 预测

        状态 X = [x_c, y_c, z_c, vx_c, vy_c, vz_c, yaw, ω, r1, r2, dz]^T (11维)
        观测 Z = [x_armor, y_armor, z_armor, yaw_armor]^T (4维)
        观测方程（非线性）：
            （都在世界坐标系下）
            P_armor = P_center + R_z(yaw) * P_body
            yaw_armor = yaw + Δyaw

                  +180  
                    |
            -90 ——> x <—— +90
                    |
                    0

        其中 P_body 是 四个装甲板相对于 整车中心 的坐标，R_z(yaw) 是绕z轴旋转 yaw 的旋转矩阵，也就是装甲板相对于中心的坐标经过旋转后得到装甲板在世界坐标系中的位置。
        Δyaw 取决于装甲板ID，比如 ID=1 Δyaw = +180°，ID=2 Δyaw = +90°，ID3 Δyaw = 0°，ID4 Δyaw = -90°。
*/

#ifndef EKF_HPP
#define EKF_HPP

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>  
#include <memory>
#include <vector>
#include <string>
#include <deque>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

/**
 * @brief Extended Kalman Filter (EKF) for Armor Tracking
 * @details 
 * 思路：根据观测数据推算中心位姿，进行 YPD 球坐标系卡尔曼滤波，并发布从中心到四周装甲板的动态 TF 变换。
 * 状态 X = [x_c, y_c, z_c, vx_c, vy_c, vz_c, yaw, ω, r1, r2, dz]^T (11维)
 * 观测 Z = [los_yaw, los_pitch, distance, yaw_armor]^T (4维 YPD 球面坐标观测)
 */
class EKF
{
public:

    // ==================== 构造与重置初始化 ====================

    explicit EKF(rclcpp::Node* node); // 构造函数拿到 Node 指针以创建 ROS2 相关的对象

    void Reset(); // 重置初始化（丢失目标超过一定帧数时调用）

    bool is_initialized_; // 滤波器是否已完成首次初始化

    // ==================== 参数与配置接口 ====================

    void UpdateParamsFromServer();                  // 从参数服务器动态更新 Q、R 矩阵参数
    void SetArmorplateSize(std::string ARMOR_TYPE); // 根据识别，设置装甲板的物理尺寸 大小板 (宽和高)
    void SetArmorNum(int num);                      // 根据识别，设置一个车上装甲板的数量 (兵种区分)

    // ==================== 核心算法接口【流水线】 ====================

    // 1. 根据首次观测数据完成 EKF 的状态初始化
    void Initialized(const Eigen::Vector3d& armorplate_center, double yaw_armor, int id, rclcpp::Time current_image_time); 

    // 2. 状态预测，将状态通过预测推演到当前时间
    void PredictState(double dt);

    // 3. 状态更新，将状态通过观测数据更新
    void UpdateState(const Eigen::Vector3d& armorplate_center, double yaw, int id, rclcpp::Time current_image_time);

    // ==================== 物理发散 与 NIS崩溃检测 接口 ====================
    
    bool IsDiverged() const;  // 物理发散检测
    bool IsNISFailed() const; // NIS 崩溃检测

    // ==================== 状态提取接口 ====================

    // 预测未来 future 秒，整车上指定 ID 装甲板的位置。通过传入引用 armorplate_center_predict 得到坐标 x y z
    void GetArmorplatePredict(Eigen::Vector3d& armorplate_center_predict, int armor_id, double future_time);
    
    // 预测未来 future 秒，整车中心点的位置。通过传入引用 car_center_predict 得到坐标 x y z
    void GetCarCenterPredict(Eigen::Vector3d& car_center_predict, double future_time);

    // 获取现在，指定 ID 装甲板的四个角点在世界系的坐标（根据现在整车状态来推，因此坐标系无需担心）
    void GetArmorplateFourCorners(std::vector<Eigen::Vector3d>& corners, int armor_id); 

    // ==================== 状态矩阵 ====================

    Eigen::Matrix<double, 11, 1> X;      // k 时刻状态量
    // Eigen::Matrix<double, 11, 1> X_est;  // k 时刻预测状态量
    // Eigen::Matrix<double, 11, 1> X_prev; // k-1 时刻状态量

private:

    // ==================== ROS & TF 基础组件 ====================

    rclcpp::Node* node_; // ROS2 节点指针

    std::unique_ptr<tf2_ros::TransformBroadcaster> car_broadcaster_;  // 发布 car_center_frame -> armor_x_frame 的动态变换，即目前整车中心到四个装甲板的变换
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                      // TF 缓存
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;         // TF 监听器

    // ==================== EKF 内部逻辑流程 ====================
    
    void UpdateParameters(double dt); // 依据 dt 刷新状态转移矩阵 F 和过程噪声 Q

    // ==================== NIS 卡方检验相关，验证观测数据是否异常 ====================
    
    std::deque<bool> recent_nis_failures_; // 引入同济的滑动窗口机制
    int window_size_ = 10; // 窗口大小设为10帧
    int max_recent_nis_failures_ = 4; // 窗口内最多允许 4 次NIS检验失败，超过则认为当前观测异常

    // ==================== 观测模型 H 与雅可比 h ====================

    Eigen::Matrix<double, 4, 1> h(const Eigen::Matrix<double, 11, 1>& x);               // XYZ 空间非线性观测方程
    Eigen::Matrix<double, 4, 11> ComputeH(const Eigen::Matrix<double, 11, 1>& x);       // XYZ 空间雅可比矩阵

    Eigen::Matrix<double, 4, 1> h_ypd(const Eigen::Matrix<double, 11, 1>& X_in);         // YPD 极坐标空间非线性观测方程
    Eigen::Matrix<double, 4, 11> ComputeH_YPD(const Eigen::Matrix<double, 11, 1>& X_in); // YPD 极坐标空间雅可比矩阵 (链式法则)

    // ==================== 辅助与工具函数 ====================

    void NormalizeAngle(double& angle); // 将角度归一化到 [-PI, PI] 之间

    // 根据目前整车状态，计算指定 ID 装甲板的相对中心坐标 (P_body)，即得到相对中心偏差量 dx dy
    // 以及该板当前相对于整车的角度（0 号装甲板角度）的固定偏航角 Δyaw（逆时针为正）
    void GetArmorplateParams(const Eigen::Matrix<double, 11, 1>& X_in, int id, double& dx, double& dy, double& dz_offset, double& theta_offset);

    void PrintDebugInfo(Eigen::Matrix<double, 4, 1>& innovation); // 打印底层 EKF 调试日志

    // ==================== TF 发布逻辑 ====================

    // 控制发布从整车中心到指定 ID 装甲板的 TF 变换
    void UpdateCarCenterToArmorplate(std::string child_frame, double x, double y, double z, double roll, double pitch, double yaw);
    
    // 控制发布从整车中心到指定 ID 装甲板的 TF 变换总逻辑，循环发布 中心 到 armor_1_frame ~ armor_4_frame
    void UpdateCarCenterToArmorplates(); 

    // 发布 father_frame -> car_center_frame 的 TF 变换
    void UpdateFatherToCarCenter();      

    // ==================== 内部属性与参数 ====================

    bool SHOW_LOGGER_DEBUG;   // EKF 调试日志开关

    int armor_num_;  // 装甲板总数
    int armor_id_;   // 当前帧传入的观测装甲板 ID
    double width_;    // 装甲板物理宽度 (m)
    double height_;   // 装甲板物理高度 (m)

    std::string father_frame_; // 坐标系基准：单机模式(camera_frame) / 联调模式(world_frame)
    rclcpp::Time current_image_time_; // 当前图像时间戳，用于同步 TF 发布

    // ==================== 卡尔曼滤波矩阵库 ====================

    Eigen::Matrix<double, 11, 11> P;      // k 时刻协方差矩阵
    // Eigen::Matrix<double, 11, 11> P_est;  // k 时刻预测协方差矩阵
    // Eigen::Matrix<double, 11, 11> P_prev; // k-1 时刻协方差矩阵

    Eigen::Matrix<double, 11, 11> F;      // 状态转移矩阵
    Eigen::Matrix<double, 11, 11> Q;      // 预测过程噪声协方差矩阵
    Eigen::Matrix<double, 4, 11>  H;      // 观测雅可比矩阵 (∂h/∂x)
    Eigen::Matrix<double, 4, 4>   R;      // 测量过程噪声协方差矩阵
    Eigen::Matrix<double, 4, 1>   Z;      // 实时测量向量 (YPD + 欧拉角yaw)
    Eigen::Matrix<double, 11, 4>  K;      // 卡尔曼增益矩阵
    Eigen::Matrix<double, 11, 11> I;      // 单位矩阵

    // ==================== 动态调参变量 (Yaml) ====================

    double q_x_, q_y_, q_z_;
    double q_v_x_, q_v_y_, q_v_z_;
    double q_yaw_, q_omega_;
    double q_r_, q_dz_; 

    double q_v1_; // 平移加速度方差（白噪声模型）
    double q_v2_; // 旋转角加速度方差（白噪声模型）

    double r_los_yaw_;   // 视线偏航角观测噪声
    double r_los_pitch_; // 视线俯仰角观测噪声
    double r_distance_;  // 深度距离观测噪声 (单目 PnP 通常较大)
    double r_euler_yaw_; // 目标装甲板绝对姿态观测噪声
};

#endif // EKF_HPP
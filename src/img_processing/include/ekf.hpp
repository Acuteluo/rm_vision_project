#pragma once
#include <rclcpp/rclcpp.hpp>
#include<Eigen/Dense>
#include<Eigen/QR>
#include<Eigen/Core>
#include<Eigen/LU>
#include <cmath>  

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


/*
    EKF 

        思路：目前先固定R，根据R发布动态变换（传一次观测数据就发布一次），发布从中心到四周装甲板的变换（十字架模型）

             传入观测数据 -> 推算几何中心位姿 -> ekf滤波 -> 发布整车中心与四个装甲板的坐标系 -> 预测

        状态 X = [x_c, y_c, z_c, vx_c, vy_c, vz_c, yaw, ω, a_ω]^T (9维)
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


class EKF
{
public:
// 构造函数拿到 Node 指针以创建 ROS2 相关的对象
    explicit EKF(rclcpp::Node* node); 
	

	/**
	* @brief	滤波主要过程
	* @param	x
	* @param	y
    * @param    z 
    * @param    dt
	* @return   无返回值
	*/
	void getKalman(Eigen::Vector3d armorplate_center, double yaw_armor, int armor_id, double dt);
	

	// 初始化  状态转移矩阵F  协方差矩阵P  预测过程噪声Q  观测矩阵H  测量过程噪声R
	void Initialized();
	
    // 重置初始化（丢失目标超过一定帧数时）
    void reset();

	// 更新参数
	void CalculateParameter(double dt);
	


	// 状态预测 X_hat_k_est = F * X_hat_k-1
	void StatusPredict();
	


	// 不确定性预测 P_k_est = F * P_k-1 * F^T + Q
	void UncertaintyPredict();
	

	// 计算卡尔曼增益 K_k = P_k_est * H^T * [H * P_k_est * H^T + R]^-1
	void CalculateKalmanGain();
	


	// 用测量值更新状态 X_hat_k = X_hat_k_est + K_k * (Z_k - H * X^k_est)
	void UpdateStatus();
	


	// 更新协方差，不确定性 P_k = (I - K_k * H) * P_k_est
	void UpdateUncertainty();
	


	// 更新历史数据
	void UpdateHistoricalData();


	// 获得装甲板的预测位置。通过传入引用，获得 x y z
    void getArmorPredict(Eigen::Vector3d& armorplate_center_predict, 
                          int armor_id, double future_time);


	// 预测未来 future 秒的中心点的位置，通过传入引用，获得 x y z
	void getCenterPredict(Eigen::Vector3d& car_center_predict, double future_time);
	

    // 改变滤波器内部状态的预测，会更新状态
    void predictOnly(double dt);


    // 05【整车中心坐标系】-> 四个【装甲板坐标系】
    void updateCarCenterToArmorplate(std::string child_frame, double x, double y, double z, double roll, double pitch, double yaw);

    void updateFourArmorplates();



    // 查询【世界坐标系】-> 【整车中心坐标系】
    // bool getTransform(double& x_c, double& y_c, double& z_c, double& yaw);

private:

    // 非线性观测方程 h(x, armor_id)：将状态映射到装甲板观测空间
    Eigen::Matrix<double, 4, 1> h(const Eigen::Matrix<double, 9, 1>& x);

    // 计算观测雅可比矩阵 H = ∂h/∂x，在预测状态 X_est 处线性化
    Eigen::Matrix<double, 4, 9> computeH(const Eigen::Matrix<double, 9, 1>& x);

    // 根据装甲板 ID 获取车体系下偏移量 (dx, dy) 和偏航角差 θ_offset
    void getArmorParams(double& dx, double& dy, double& theta_offset);


    double radius = 0.25; // 整车旋转半径m

	// 状态矩阵
	Eigen::Matrix<double, 9, 1> X;      // k 时刻状态
	Eigen::Matrix<double, 9, 1> X_est;  // k 时刻预测状态
	Eigen::Matrix<double, 9, 1> X_prev; // k-1 时刻状态

	// 协方差矩阵
	Eigen::Matrix<double, 9, 9> P;      // k 时刻协方差矩阵
	Eigen::Matrix<double, 9, 9> P_est;  // k 时刻预测协方差矩阵
	Eigen::Matrix<double, 9, 9> P_prev; // k-1 时刻协方差矩阵

	// 状态转移矩阵 
	Eigen::Matrix<double, 9, 9> F;

	// 预测过程噪声矩阵
	Eigen::Matrix<double, 9, 9> Q;

	// 观测矩阵（不写定值，而是雅可比矩阵） 只观测 x_center y_center z_center yaw
	Eigen::Matrix<double, 4, 9> H;

	// 测量过程噪声 
	Eigen::Matrix<double, 4, 4> R;

	// 测量矩阵 测量 xyz
	Eigen::Matrix<double, 4, 1> Z;

	// 卡尔曼增益
	Eigen::Matrix<double, 9, 4> K;

	// 单位矩阵
	Eigen::Matrix<double, 9, 9> I;

	bool is_initialized; // 是否初始化

    // TF 广播器、缓存、监听器，发布从 整车中心到装甲板的动态变换
    // car_center_frame -> armorplate_frame
    std::unique_ptr<tf2_ros::TransformBroadcaster> car_broadcaster_;     // 发布 car_center_frame -> armorplate_flame 相关 TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                 // TF 缓存
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;    // TF 监听器

    rclcpp::Node* node_;  // 拿到 ros2 的节点指针

    int armor_id; // 传入的装甲板 id （1:前，2:右，3:后，4:左）

};

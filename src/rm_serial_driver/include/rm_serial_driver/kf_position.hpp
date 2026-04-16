#pragma once
#include<Eigen/Dense>
#include<Eigen/QR>
#include<Eigen/Core>
#include<Eigen/LU>


namespace rm_serial_driver
{
// 匀速卡尔曼滤波 状态 [x y z v_x v_y v_z]^T
class KalmanFilter
{
public:
	KalmanFilter();
	

	/**
	* @brief	滤波主要过程
	* @param	x
	* @param	y
    * @param    z 
    * @param    dt
	* @return   无返回值
	*/
	void getKalman(double x, double y, double z, double dt);
	

	// 初始化  状态转移矩阵F  协方差矩阵P  预测过程噪声Q  观测矩阵H  测量过程噪声R
	void Initialized();
	


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
	
	

	// 获得当前帧的预测位置。通过传入引用，获得 x y z
	void getData(double& X_get, double& Y_get, double& Z_get);
	


	// 预测未来 future 秒的位置，通过传入引用，获得 x y z
	void getPredict(double& X_get, double& Y_get, double& Z_get, double future_time);
	


private:

	// 状态矩阵
	Eigen::Matrix<double, 6, 1> X;      // k 时刻状态[x y z v_x v_y v_z]
	Eigen::Matrix<double, 6, 1> X_est;  // k 时刻预测状态[x y z]
	Eigen::Matrix<double, 6, 1> X_prev; // k-1 时刻状态[x y z v_x v_y v_z]

	// 协方差矩阵
	Eigen::Matrix<double, 6, 6> P;      // k 时刻协方差矩阵
	Eigen::Matrix<double, 6, 6> P_est;  // k 时刻预测协方差矩阵
	Eigen::Matrix<double, 6, 6> P_prev; // k-1 时刻协方差矩阵

	// 状态转移矩阵 
	Eigen::Matrix<double, 6, 6> F;

	// 预测过程噪声矩阵
	Eigen::Matrix<double, 6, 6> Q;

	// 观测矩阵 只观测 xyz 位置
	Eigen::Matrix<double, 3, 6> H;

	// 测量过程噪声 
	Eigen::Matrix<double, 3, 3> R;

	// 测量矩阵 测量 xyz
	Eigen::Matrix<double, 3, 1> Z;

	// 卡尔曼增益
	Eigen::Matrix<double, 6, 3> K;

	// 单位矩阵
	Eigen::Matrix<double, 6, 6> I;

	bool is_initialized; // 是否初始化

};

}
#include <kf_data.hpp>


namespace rm_serial_driver
{

// 匀速卡尔曼滤波 状态 [pitch yaw]^T
KF::KF()
{
	this->is_initialized = false; // 没有初始化
	I = Eigen::Matrix<double, 2, 2>::Identity(); // 2*2 单位矩阵
}



void KF::getKalman(double pitch, double yaw)
{

	// 如果没有初始化就初始化
	if (this->is_initialized == false)
	{
		Initialized();
		X_prev << pitch, yaw;   // 初始是当前 pitch 和 yaw
        X = X_prev;        
        return; // 第一帧不输出结果
	}

	// 更新参数
	CalculateParameter(); 

	///////////////////////////////////// 卡尔曼五步 //////////////////////////////////////

	StatusPredict(); // 状态预测

	UncertaintyPredict(); // 不确定性预测

	CalculateKalmanGain(); // 计算卡尔曼增益

	Z << pitch, yaw; // 写入测量值[pitch yaw]
	UpdateStatus(); // 用测量值更新状态

	UpdateUncertainty(); // 更新不确定性

	UpdateHistoricalData(); // 更新历史数据

}



// 初始化  状态转移矩阵F  协方差矩阵P  预测过程噪声Q  观测矩阵H  测量过程噪声R
void KF::Initialized()
{

	// 协方差矩阵 位置 + 速度
	P_prev << 0.1, 0, 
		    0, 0.1;


	// 测量过程噪声 z 方向可能最不准
	R << 0.5, 0, 
        0, 0.5;

	this->is_initialized = true;
}


// 更新参数
void KF::CalculateParameter()
{
	// 观测矩阵 观测 pitch yaw
	H << 1, 0,
        0, 1;


	// 状态转移矩阵
	// pitch_k = pitch_k-1
	// yaw_k = yaw_k-1
	F << 1, 0,
        0, 1;


	// 预测过程噪声矩阵
	Q << 0.001, 0,
        0, 0.001;

}


// 状态预测 X_hat_k_est = F * X_hat_k-1
void KF::StatusPredict()
{
	X_est = F * X_prev;
}


// 不确定性预测 P_k_est = F * P_k-1 * F^T + Q
void KF::UncertaintyPredict()
{
	P_est = F * P_prev * F.transpose() + Q;
}


// 计算卡尔曼增益 K_k = P_k_est * H^T * [H * P_k_est * H^T + R]^-1
void KF::CalculateKalmanGain()
{
	K = P_est * H.transpose() * (H * P_est * H.transpose() + R).inverse();
}


// 用测量值更新状态 X_hat_k = X_hat_k_est + K_k * (Z_k - H * X^k_est)
void KF::UpdateStatus()
{
	X = X_est + K * (Z - H * X_est);
}


// 更新协方差，不确定性 P_k = (I - K_k * H) * P_k_est
void KF::UpdateUncertainty()
{
	P = (I - K * H) * P_est;
}


// 更新历史数据
void KF::UpdateHistoricalData()
{
	X_prev = X;
	P_prev = P;
}


// 获得当前帧的预测位置。通过传入引用，获得 pitch yaw
void KF::getData(double& pitch_get, double& yaw_get)
{
	pitch_get = this->X(0);
    yaw_get = this->X(1);
}

}
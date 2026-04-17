// #include <rm_serial_driver/ekf.hpp>


// namespace rm_serial_driver
// {
// // 匀速卡尔曼滤波 状态 [x y z v_x v_y v_z]^T
// EKF::EKF()
// {
// 	this->is_initialized = false; // 没有初始化
// 	I = Eigen::Matrix<double, 6, 6>::Identity(); // 6*6 单位矩阵
// }



// void EKF::getKalman(double x, double y, double z, double dt)
// {

// 	// 如果没有初始化就初始化
// 	if (this->is_initialized == false)
// 	{
// 		Initialized();
// 		X_prev << x, y, z, 0.0, 0.0, 0.0;   // 初始位置是当前位置，速度 0 
//         X = X_prev;        
//         return; // 第一帧不输出结果
// 	}

// 	// 根据dt更新参数
// 	CalculateParameter(dt); 

// 	///////////////////////////////////// 卡尔曼五步 //////////////////////////////////////

// 	StatusPredict(); // 状态预测

// 	UncertaintyPredict(); // 不确定性预测

// 	CalculateKalmanGain(); // 计算卡尔曼增益

// 	Z << x, y, z; // 写入测量值[x y z]
// 	UpdateStatus(); // 用测量值更新状态

// 	UpdateUncertainty(); // 更新不确定性

// 	UpdateHistoricalData(); // 更新历史数据

// }



// // 初始化  状态转移矩阵F  协方差矩阵P  预测过程噪声Q  观测矩阵H  测量过程噪声R
// void EKF::Initialized()
// {

// 	// 协方差矩阵 位置 + 速度
// 	P_prev << 0.1, 0, 0, 0, 0, 0,    
// 		    0, 0.1, 0, 0, 0, 0,
//             0, 0, 0.1, 0, 0, 0,
//             0, 0, 0, 1.0, 0, 0,
//             0, 0, 0, 1.0, 0, 0,
//             0, 0, 0, 1.0, 0, 0;


// 	// 测量过程噪声 z 方向可能最不准
// 	R << 0.1, 0, 0,
//         0, 0.1, 0,
//         0, 0, 0.3;

// 	this->is_initialized = true;
// }


// // 依据 dt 更新参数
// void EKF::CalculateParameter(double dt)
// {
// 	// 观测矩阵 只观测位置
// 	H << 1, 0, 0, 0, 0, 0,
//         0, 1, 0, 0, 0, 0,
//         0, 0, 1, 0, 0, 0;


// 	// 状态转移矩阵
// 	// x_k = x_k-1 + v_x * dt
// 	// y_k = y_k-1 + v_y * dt
//     // z_k = z_k-1 + v_z * dt
//     // v_x_k = v_x_k-1 
//     // v_y_k = v_y_k-1 
//     // v_z_k = v_z_k-1 
// 	F << 1, 0, 0, dt, 0, 0,
//         0, 1, 0, 0, dt, 0,
//         0, 0, 1, 0, 0, dt,
//         0, 0, 0, 1, 0, 0,
//         0, 0, 0, 0, 1, 0,
//         0, 0, 0, 0, 0, 1;


// 	// 预测过程噪声矩阵
// 	Q << 0.0001, 0, 0, 0, 0, 0,
//         0, 0.0001, 0, 0, 0, 0,
//         0, 0, 0.0001, 0, 0, 0,
//         0, 0, 0, 0.005, 0, 0,
//         0, 0, 0, 0, 0.005, 0,
//         0, 0, 0, 0, 0, 0.005;

// }


// // 状态预测 X_hat_k_est = F * X_hat_k-1
// void EKF::StatusPredict()
// {
// 	X_est = F * X_prev;
// }


// // 不确定性预测 P_k_est = F * P_k-1 * F^T + Q
// void EKF::UncertaintyPredict()
// {
// 	P_est = F * P_prev * F.transpose() + Q;
// }


// // 计算卡尔曼增益 K_k = P_k_est * H^T * [H * P_k_est * H^T + R]^-1
// void EKF::CalculateKalmanGain()
// {
// 	K = P_est * H.transpose() * (H * P_est * H.transpose() + R).inverse();
// }


// // 用测量值更新状态 X_hat_k = X_hat_k_est + K_k * (Z_k - H * X^k_est)
// void EKF::UpdateStatus()
// {
// 	X = X_est + K * (Z - H * X_est);
// }


// // 更新协方差，不确定性 P_k = (I - K_k * H) * P_k_est
// void EKF::UpdateUncertainty()
// {
// 	P = (I - K * H) * P_est;
// }


// // 更新历史数据
// void EKF::UpdateHistoricalData()
// {
// 	X_prev = X;
// 	P_prev = P;
// }


// // 获得当前帧的预测位置。通过传入引用，获得 x y z
// void EKF::getData(double& X_get, double& Y_get, double& Z_get)
// {
// 	X_get = this->X(0);
//     Y_get = this->X(1);
//     Z_get = this->X(2);
// }



// // 预测未来 future 秒的位置，通过传入引用，获得 x y z
// void EKF::getPredict(double& X_get, double& Y_get, double& Z_get, double future_time)
// {
// 	X_get = this->X(0) + this->X(3) * future_time; // x + v_x * future_time
//     Y_get = this->X(1) + this->X(4) * future_time; // y + v_y * future_time
//     Z_get = this->X(2) + this->X(5) * future_time; // z + v_z * future_time
// }

// }
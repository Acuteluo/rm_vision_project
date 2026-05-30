#include <ekf.hpp>


EKF::EKF(rclcpp::Node* node): node_(node)
{
	this->is_initialized = false; // 没有初始化

    car_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
	
    // 创建 TF 缓存和监听器（用来查询【世界坐标系 -> 整车中心坐标系】是否可以变换）
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    I = Eigen::Matrix<double, 9, 9>::Identity(); // 9*9 单位矩阵
}



// 从参数服务器更新（运行时调用）
void EKF::UpdateParamsFromServer() 
{
    if (!node_) return;

    // 预测矩阵 q
    this->q_x_ = node_->get_parameter("ekf.q_x").as_double();
    this->q_y_ = node_->get_parameter("ekf.q_y").as_double();
    this->q_z_ = node_->get_parameter("ekf.q_z").as_double();
    this->q_v_x_ = node_->get_parameter("ekf.q_v_x").as_double();
    this->q_v_y_ = node_->get_parameter("ekf.q_v_y").as_double();
    this->q_v_z_ = node_->get_parameter("ekf.q_v_z").as_double();
    this->q_yaw_ = node_->get_parameter("ekf.q_yaw").as_double();
    this->q_omega_ = node_->get_parameter("ekf.q_omega").as_double();
    this->q_a_omega_ = node_->get_parameter("ekf.q_a_omega").as_double();

    // 观测矩阵 r
    this->r_los_yaw_ = node_->get_parameter("ekf.r_los_yaw").as_double();
    this->r_los_pitch_ = node_->get_parameter("ekf.r_los_pitch").as_double();
    this->r_distance_ = node_->get_parameter("ekf.r_distance").as_double();
    this->r_euler_yaw_ = node_->get_parameter("ekf.r_euler_yaw").as_double();

    // 整车半径
    this->radius = node_->get_parameter("ekf.radius").as_double(); 

    // 父坐标系名称，即发布谁到整车中心的变换。单机模式下是 camera_frame，联调模式下是 world_frame
    this->father_frame = node_->get_parameter("core.mode.is_standalone_mode").as_bool() ? "camera_frame" : "world_frame";

    // 是否打印调参日志
    this->SHOW_LOGGER_DEBUG = node_->get_parameter("ekf.show_logger_debug").as_bool();
}
    



// 设置装甲板的 width 和 height
void EKF::SetArmorplateSize(std::string ARMOR_TYPE)
{
    if(ARMOR_TYPE == "normal") // 步兵装甲板
    {
        this->width = 0.135;
        this->height = 0.055;
    }
    else //英雄装甲板
    {
        this->width = 0.225;
        this->height = 0.055;
    }
}


// 设置是否打印日志
void EKF::SetDebugLogger(bool SHOW_LOGGER_DEBUG)
{
    this->SHOW_LOGGER_DEBUG = SHOW_LOGGER_DEBUG;
}

// 设置装甲板的数量
void EKF::SetArmorNum(int num) 
{ 
    armor_num_ = num; 
} 


void EKF::Reset()
{
    this->is_initialized = false; // 重置初始化
}


// 05【整车中心坐标系】-> 四个【装甲板坐标系】
void EKF::UpdateCarCenterToArmorplate(std::string child_frame, double x, double y, double z, double roll, double pitch, double yaw)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->node_->now(); // 帧头 -> 直接获取当前时刻
    tf.header.frame_id = "car_center_frame"; // 父坐标系 -> 整车中心坐标系
    tf.child_frame_id = child_frame; // 子坐标系 -> 四个装甲板坐标系

    // 平移
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    
    // 旋转
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);  // 顺序：roll, pitch, yaw （XYZ） 

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    // 设置四元数
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    car_broadcaster_->sendTransform(tf);
}


void EKF::UpdateCarCenterToArmorplates()
{
    for (int i = 0; i < armor_num_; i++) 
    {
        double offset = i * 2.0 * M_PI / armor_num_;
        NormalizeAngle(offset);
        double dx = -this->radius * std::cos(offset);
        double dy = -this->radius * std::sin(offset);
        
        std::string frame_name = "armor_" + std::to_string(i);
        UpdateCarCenterToArmorplate(frame_name, dx, dy, 0.0, 0.0, 0.0, offset);
    }
}


// 05 【父坐标系】->【整车中心坐标系】注意这里忽略了 pitch & roll
// 如果是单机模式，父坐标系是 camera_frame，如果是联调模式，父坐标系是 world_frame
void EKF::UpdateFatherToCarCenter()
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->node_->now(); // 帧头 -> 直接获取当前时刻
    tf.header.frame_id = this->father_frame; // 父坐标系 -> 相机坐标系 / 世界坐标系
    tf.child_frame_id = "car_center_frame"; // 子坐标系 -> 整车中心坐标系

    // 平移
    tf.transform.translation.x = this->X(0);
    tf.transform.translation.y = this->X(1);
    tf.transform.translation.z = this->X(2);
    
    // 旋转 这里忽略 pitch & roll
    tf2::Quaternion q;
    q.setRPY(0, 0, this->X(6));  // 顺序：roll, pitch, yaw （XYZ） 

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    // 设置四元数
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    car_broadcaster_->sendTransform(tf);
}


// 根据 ID 获取装甲板在 整车中心坐标系 下的参数
void EKF::GetArmorplateParams(double& dx, double& dy, double& theta_offset)
{
    // 严格按照你的 0123 体系：偏差角 = ID * (360 / 装甲板数量)
    theta_offset = this->armor_id * 2.0 * M_PI / armor_num_;

    // 装甲板的 +X 轴指向车心，所以它处于车心的反方向
    dx = -this->radius * std::cos(theta_offset);
    dy = -this->radius * std::sin(theta_offset);
}


// 这是滤波的实时值，而不是预测位置喵
// 得到某个 id 装甲板四个角点在世界下的坐标
void EKF::GetArmorplateFourCorners(std::vector<Eigen::Vector3d>& corners, int armor_id)
{
    corners.resize(4);
    double x_c = this->X(0), y_c = this->X(1), z_c = this->X(2), yaw = this->X(6);
    this->armor_id = armor_id;
    double dx, dy, theta_offset;
    GetArmorplateParams(dx, dy, theta_offset);

    // 根据 ARMOR_TYPE 实际值，单位m
    const double width = this->width;   
    const double height = this->height;

    double cos_yaw = cos(yaw), sin_yaw = sin(yaw);
    double cx = x_c + cos_yaw * dx - sin_yaw * dy;
    double cy = y_c + sin_yaw * dx + cos_yaw * dy;
    double cz = z_c;
    double armor_yaw = yaw + theta_offset;

    double half_w = width / 2.0, half_h = height / 2.0;
    double cos_ay = cos(armor_yaw), sin_ay = sin(armor_yaw);

    // 局部坐标 (y, z) 偏移 (法向为 x，所以角点在 x=0 平面上)
    std::pair<double, double> offsets[4] = {
        {-half_w,  half_h},  // 左上
        { half_w,  half_h},  // 右上
        { half_w, -half_h},  // 右下
        {-half_w, -half_h}   // 左下
    };

    for (int i = 0; i < 4; ++i) 
    {
        double local_y = offsets[i].first;
        double local_z = offsets[i].second;
        double wx = cx + cos_ay * 0 - sin_ay * local_y;
        double wy = cy + sin_ay * 0 + cos_ay * local_y;
        double wz = cz + local_z;
        corners[i] = Eigen::Vector3d(wx, wy, wz);
    }
}


// 非线性观测方程 h(x, armor_id)：将 整车预测的状态量XYZ 映射到 装甲板观测空间XYZ
Eigen::Matrix<double, 4, 1> EKF::h(const Eigen::Matrix<double, 9, 1>& X_in)
{
    double x_c = X_in(0), y_c = X_in(1), z_c = X_in(2);
    double yaw = X_in(6);
    double dx, dy, theta_offset;
    GetArmorplateParams(dx, dy, theta_offset);

    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    // 装甲板坐标系 在整车中心坐标系下的坐标

    Eigen::Matrix<double, 4, 1> z_pred;
    z_pred(0) = x_c + cos_yaw * dx - sin_yaw * dy;   // x_armor
    z_pred(1) = y_c + sin_yaw * dx + cos_yaw * dy;   // y_armor
    z_pred(2) = z_c;                                 // z 相同
    z_pred(3) = yaw + theta_offset;                  // yaw_armor
    NormalizeAngle(z_pred(3)); // 归一化装甲板的 yaw 到 [-pi, pi]

    return z_pred;
}


// 观测矩阵 雅可比矩阵 H = ∂h/∂x
Eigen::Matrix<double, 4, 9> EKF::ComputeH(const Eigen::Matrix<double, 9, 1>& X_in)
{
    double yaw = X_in(6);
    double dx, dy, theta_offset;
    GetArmorplateParams(dx, dy, theta_offset);  // theta_offset 用不到，只关心 dx, dy

    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    Eigen::Matrix<double, 4, 9> H_mat = Eigen::Matrix<double, 4, 9>::Zero();

    // 对 x_c, y_c, z_c 的偏导
    H_mat(0, 0) = 1.0;
    H_mat(1, 1) = 1.0;
    H_mat(2, 2) = 1.0;

    // 对 yaw 的偏导
    // ∂x_armor/∂yaw = -sin(yaw)*dx - cos(yaw)*dy
    H_mat(0, 6) = -sin_yaw * dx - cos_yaw * dy;
    // ∂y_armor/∂yaw =  cos(yaw)*dx - sin(yaw)*dy
    H_mat(1, 6) =  cos_yaw * dx - sin_yaw * dy;
    // ∂yaw_armor/∂yaw = 1
    H_mat(3, 6) = 1.0;

    // 其余偏导（对速度、角速度）均为 0
    return H_mat;
}



///////////////////////////// YPD 相关函数的实现 ///////////////////////////////////

// 【新增】：将状态量直接映射为 YPD 观测量的非线性函数
// 用当前预测的整车状态 X_est​ (XYZ) 反推出来的预测值 YPD
Eigen::Matrix<double, 4, 1> EKF::h_ypd(const Eigen::Matrix<double, 9, 1>& X_in)
{
    // 1. 先调用原来的 h() 函数，得到相机系下的 XYZ 预测 和 Yaw_armor 预测
    Eigen::Matrix<double, 4, 1> z_xyz = this->h(X_in); 
    double pred_x = z_xyz(0);
    double pred_y = z_xyz(1);
    double pred_z = z_xyz(2);
    double pred_yaw_armor = z_xyz(3);

    // 2. 将 XYZ 转换为 YPD 球面坐标系 (FLU 右手系和 armorplate.cpp 的公式)
    double r2 = pred_x * pred_x + pred_y * pred_y;
    double d = std::sqrt(r2 + pred_z * pred_z);

    Eigen::Matrix<double, 4, 1> z_ypd;
    z_ypd(0) = std::atan2(pred_y, pred_x);              // los_yaw
    z_ypd(1) = -std::atan2(pred_z, std::sqrt(r2));      // los_pitch
    z_ypd(2) = d;                                       // distance
    z_ypd(3) = pred_yaw_armor;                          // 装甲板自身 yaw 不变

    return z_ypd;
}


// 【新增】：利用链式求导法则，算出 YPD 坐标系下的 4x9 雅可比矩阵
Eigen::Matrix<double, 4, 9> EKF::ComputeH_YPD(const Eigen::Matrix<double, 9, 1>& X_in)
{
    // 1. 先复用你原来写好的 XYZ 雅可比矩阵 H_xyz (4x9)
    Eigen::Matrix<double, 4, 9> H_xyz = this->ComputeH(X_in);

    // 2. 获取预测的 XYZ
    Eigen::Matrix<double, 4, 1> z_xyz = this->h(X_in);
    double x = z_xyz(0), y = z_xyz(1), z = z_xyz(2);

    double r2 = x * x + y * y;
    double r = std::sqrt(r2);
    double d2 = r2 + z * z;
    double d = std::sqrt(d2);

    // 3. 构建 YPD 对 XYZ 的 3x3 微分矩阵 J_ypd
    Eigen::Matrix3d J_ypd = Eigen::Matrix3d::Zero();
    
    // 防止除零导致矩阵爆炸
    if (r2 > 1e-6 && d2 > 1e-6) 
    {
        // ∂yaw_cam / ∂(x,y,z)
        J_ypd(0, 0) = -y / r2;
        J_ypd(0, 1) =  x / r2;
        J_ypd(0, 2) =  0.0;

        // ∂pitch_cam / ∂(x,y,z) (严格求导结果)
        J_ypd(1, 0) = (z * x) / (d2 * r);
        J_ypd(1, 1) = (z * y) / (d2 * r);
        J_ypd(1, 2) = -r / d2;

        // ∂distance / ∂(x,y,z)
        J_ypd(2, 0) = x / d;
        J_ypd(2, 1) = y / d;
        J_ypd(2, 2) = z / d;
    }

    // 4. 链式法则魔法：H_new_top3 = J_ypd * H_xyz_top3
    Eigen::Matrix<double, 4, 9> H_new = Eigen::Matrix<double, 4, 9>::Zero();
    H_new.block<3, 9>(0, 0) = J_ypd * H_xyz.block<3, 9>(0, 0); 
    H_new.row(3) = H_xyz.row(3); // 第 4 行 (装甲板yaw的导数) 保持不变

    return H_new;
}


void EKF::NormalizeAngle(double& angle)
{
    angle = std::atan2(std::sin(angle), std::cos(angle));
}



// 发布整车的 tf 动态变换，拿到整车中心作为观测数据，进行预测
void EKF::UpdateExtendedKalman(Eigen::Vector3d armorplate_center, double yaw_armor, int armor_id, double dt)
{
    this->armor_id = armor_id; // 传入的装甲板 id （1:前，2:右，3:后，4:左）

	// 如果没有初始化就初始化 
    if (this->is_initialized == false)
	{
        UpdateParamsFromServer();  // 确保读取有效值

		Initialized(armorplate_center, yaw_armor);
        
        return; // 第一帧不输出结果
	}

	// 根据dt更新参数
	UpdateParameters(dt); 

    // 【修改】：把你的 Z 赋值改为存入 YPD！
    double obs_x = armorplate_center[0];
    double obs_y = armorplate_center[1];
    double obs_z = armorplate_center[2];

    double obs_los_yaw = std::atan2(obs_y, obs_x);
    double obs_los_pitch = -std::atan2(obs_z, std::sqrt(obs_x * obs_x + obs_y * obs_y));
    double obs_d = std::sqrt(obs_x * obs_x + obs_y * obs_y + obs_z * obs_z);

    // 顺序必须是：[yaw_cam, pitch_cam, distance, yaw_armor]
    Z << obs_los_yaw, obs_los_pitch, obs_d, yaw_armor;

	///////////////////////////////////// 卡尔曼五步 //////////////////////////////////////

	StatusPredict(); // 状态预测

	UncertaintyPredict(); // 不确定性预测

    // 计算当前原始观测值的 雅可比矩阵（偏导数矩阵），在 X_est 处求值，再转换到 YPD 球面坐标系下
    H = ComputeH_YPD(X_est);

	CalculateKalmanGain(); // 计算卡尔曼增益

	
	UpdateStatus(); // 用测量值更新状态

	UpdateUncertainty(); // 更新不确定性

    // 偏航角归一化到 [-π, π]
    NormalizeAngle(X(6));

	UpdateHistoricalData(); // 更新历史数据

    UpdateCarCenterToArmorplates(); // 发布 整车中心 -> 四个装甲板 的 tf 坐标系变换

    UpdateFatherToCarCenter(); // 发布 父坐标系 -> 整车中心 的 tf 坐标系
}



// 初始化  状态转移矩阵F  协方差矩阵P  预测过程噪声Q  观测矩阵H  测量过程噪声R
void EKF::Initialized(const Eigen::Vector3d& armorplate_center, const double& yaw_armor)
{
	// 协方差矩阵
	P_prev = 0.1 * I;

    // 换成 YPD 球面坐标系 
	// 测量过程噪声矩阵
    // 单位m rad
	R << this->r_los_yaw_, 0, 0, 0,
        0, this->r_los_pitch_, 0, 0,
        0, 0, this->r_distance_, 0,
        0, 0, 0, this->r_euler_yaw_;

    // 根据第一次观测反算一个粗略的中心初始状态
    double dx, dy, theta_offset;
    GetArmorplateParams(dx, dy, theta_offset);
    double init_yaw = yaw_armor - theta_offset;   // 粗略的中心偏航角

    // 反推中心位置
    double cos_yaw = cos(init_yaw), sin_yaw = sin(init_yaw);
    double init_x = armorplate_center[0] - (cos_yaw * dx - sin_yaw * dy);
    double init_y = armorplate_center[1] - (sin_yaw * dx + cos_yaw * dy);
    double init_z = armorplate_center[2];

    X_prev << init_x, init_y, init_z, 0.0, 0.0, 0.0, init_yaw, 0.0, 0.0;
    X = X_prev;

	this->is_initialized = true;
}


// 依据 dt 更新参数
void EKF::UpdateParameters(double dt)
{
	// // 观测矩阵 观测 x_c y_c z_c 和 yaw
	// H << 1, 0, 0, 0, 0, 0, 0, 0,
    //     0, 1, 0, 0, 0, 0, 0, 0,
    //     0, 0, 1, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 1, 0;

	// 状态转移矩阵
	// x_k = x_k-1 + v_x * dt
	// y_k = y_k-1 + v_y * dt
    // z_k = z_k-1 + v_z * dt
    // v_x_k = v_x_k-1 
    // v_y_k = v_y_k-1 
    // v_z_k = v_z_k-1
    // yaw_k = yaw_k-1 + omega_k-1 * dt + 0.5 * a_omega_k-1 * dt^2
    // omega_k = omega_k-1 + a_omega_k-1 * dt
    // a_omega_k = a_omega_k-1
	F << 1, 0, 0, dt, 0, 0, 0, 0, 0,
        0, 1, 0, 0, dt, 0, 0, 0, 0,
        0, 0, 1, 0, 0, dt, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, dt, 0.5*dt*dt,
        0, 0, 0, 0, 0, 0, 0, 1, dt,
        0, 0, 0, 0, 0, 0, 0, 0, 1;


	// 预测过程噪声矩阵
    // 位置的预测过程噪声较小，速度的预测过程噪声较大，角度的预测过程噪声适中，角速度的预测过程噪声较大，角加速度的预测过程噪声较大
	Q << this->q_x_, 0, 0, 0, 0, 0, 0, 0, 0,
        0, this->q_y_, 0, 0, 0, 0, 0, 0, 0,
        0, 0, this->q_z_, 0, 0, 0, 0, 0, 0,
        0, 0, 0, this->q_v_x_, 0, 0, 0, 0, 0,
        0, 0, 0, 0, this->q_v_y_, 0, 0, 0, 0,
        0, 0, 0, 0, 0, this->q_v_z_, 0, 0, 0,
        0, 0, 0, 0, 0, 0, this->q_yaw_, 0, 0,
        0, 0, 0, 0, 0, 0, 0, this->q_omega_, 0,
        0, 0, 0, 0, 0, 0, 0, 0, this->q_a_omega_;


    // 同时更新 R 矩阵（因为参数可能已改变）
    R << this->r_los_yaw_, 0, 0, 0,
        0, this->r_los_pitch_, 0, 0,
        0, 0, this->r_distance_, 0,
        0, 0, 0, this->r_euler_yaw_;

}


// 状态预测 X_hat_k_est = F * X_hat_k-1
void EKF::StatusPredict()
{
	X_est = F * X_prev;
}


// 不确定性预测 P_k_est = F * P_k-1 * F^T + Q
void EKF::UncertaintyPredict()
{
	P_est = F * P_prev * F.transpose() + Q;
}


// 计算卡尔曼增益 K_k = P_k_est * H^T * [H * P_k_est * H^T + R]^-1
void EKF::CalculateKalmanGain()
{
	K = P_est * H.transpose() * (H * P_est * H.transpose() + R).inverse();
}


// 用测量值更新状态 X_hat_k = X_hat_k_est + K_k * (Z_k - H * X_k_est)
// H * X_k_est 选出有效的量，但是 ekf 非线性，需要写一个函数 h 实现，只是把 X_k_est 换成了 h(X_est)
void EKF::UpdateStatus()
{
    // 计算新息
    // h_ypd(X_est) 是用你当前预测的整车状态 X_est​ (XYZ) 反推出来的预测值 YPD (视线角和距离)，而 Z 是你实际测量到的 YPD 观测值，所以两者都是在 YPD 坐标系下的量，直接相减就对了！不需要再转换回 XYZ 坐标系了！
    Eigen::Matrix<double, 4, 1> innovation = Z - h_ypd(X_est);
    
    // 将视线角 (los_yaw, los_pitch) 和目前姿态 (yaw_armor) 全部做归一化！
    // 否则比如 从 +179 到 -179，就会跳 358 度！但是用 sin 和 cos 就可以归一化到 -PI 到 PI 内，再 atan2 反求角度
    NormalizeAngle(innovation(0)); // los_yaw 归一化
    NormalizeAngle(innovation(1)); // los_pitch 归一化
    NormalizeAngle(innovation(3)); // yaw_armor 归一化


    // 再用处理后的新息进行状态更新
    X = X_est + K * innovation;

    // 整车角度 yaw 归一化
    NormalizeAngle(X(6)); 

    if(this->SHOW_LOGGER_DEBUG)
    {
        // ---------- 临时调参日志 ----------
        RCLCPP_INFO(rclcpp::get_logger("ekf_debug"),
            "Innovation: x=%.4f, y=%.4f, z=%.4f, yaw=%.4f",
            innovation(0), innovation(1), innovation(2), innovation(3));

        RCLCPP_INFO(rclcpp::get_logger("ekf_debug"),
            "State: x=%.3f, y=%.3f, z=%.3f, vx=%.3f, vy=%.3f, vz=%.3f, yaw=%.3f, omega=%.3f",
            X(0), X(1), X(2), X(3), X(4), X(5), X(6), X(7));

        RCLCPP_INFO(rclcpp::get_logger("ekf_debug"),
            "P_diag: px=%.4f, py=%.4f, pz=%.4f, pyaw=%.4f",
            P(0,0), P(1,1), P(2,2), P(6,6));

        RCLCPP_INFO(rclcpp::get_logger("ekf_debug"),
            "K_gain: Kx=%.4f, Ky=%.4f, Kz=%.4f, Kyaw=%.4f",
            K(0,0), K(1,1), K(2,2), K(6,3));
        // ---------- 日志结束 ----------
    }
    
}


// 更新协方差，不确定性 P_k = (I - K_k * H) * P_k_est
void EKF::UpdateUncertainty()
{
	// P = (I - K * H) * P_est;
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose(); // Joseph form 形式更新协方差矩阵，数值更稳定，防止 P 变成非正定矩阵
}


// 更新历史数据
void EKF::UpdateHistoricalData()
{
	X_prev = X;
	P_prev = P;
}



// 预测未来 future 秒的中心点的位置，通过传入引用，获得 x y z
void EKF::GetCarCenterPredict(Eigen::Vector3d& car_center_predict, double future_time)
{
	car_center_predict[0] = this->X(0) + this->X(3) * future_time; // x + v_x * future_time
    car_center_predict[1] = this->X(1) + this->X(4) * future_time; // y + v_y * future_time
    car_center_predict[2] = this->X(2) + this->X(5) * future_time; // z + v_z * future_time
}

// 获得装甲板的预测位置
void EKF::GetArmorplatePredict(Eigen::Vector3d& armorplate_center_predict, 
                          int armor_id, double future_time) 
{
    // 1. 先预测未来中心状态
    double future_x_c = this->X(0) + this->X(3) * future_time; // x + v_x * future_time
    double future_y_c = this->X(1) + this->X(4) * future_time; // y + v_y * future_time
    double future_z_c = this->X(2) + this->X(5) * future_time; // z + v_z * future_time
    double future_yaw = X(6) + X(7) * future_time + 0.5 * X(8) * future_time * future_time;
    NormalizeAngle(future_yaw);

    // 2. 根据装甲板ID获取几何参数
    this->armor_id = armor_id;
    double dx, dy, theta_offset;
    GetArmorplateParams(dx, dy, theta_offset);

    // 3. 应用旋转得到装甲板世界坐标
    double cos_yaw = cos(future_yaw);
    double sin_yaw = sin(future_yaw);

    armorplate_center_predict[0] = future_x_c + cos_yaw * dx - sin_yaw * dy;
    armorplate_center_predict[1] = future_y_c + sin_yaw * dx + cos_yaw * dy;
    armorplate_center_predict[2] = future_z_c;
}


// 改变滤波器内部状态的预测，会更新状态
void EKF::PredictOnly(double dt)
{
    if (!this->is_initialized) return;

    UpdateParameters(dt);
    StatusPredict();
    UncertaintyPredict();

    X = X_est; // 将预测状态作为当前状态
    P = P_est;

    // 偏航角归一化到 [-π, π]
    NormalizeAngle(X(6));

    X_prev = X;
    P_prev = P;
}

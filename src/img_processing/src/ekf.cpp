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
void EKF::updateParamsFromServer() 
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
    this->r_x_ = node_->get_parameter("ekf.r_x").as_double();
    this->r_y_ = node_->get_parameter("ekf.r_y").as_double();
    this->r_z_ = node_->get_parameter("ekf.r_z").as_double();
    this->r_yaw_ = node_->get_parameter("ekf.r_yaw").as_double();

    // 整车半径
    this->radius = node_->get_parameter("ekf.radius").as_double();
}
    



// 设置装甲板的 width 和 height
void EKF::setParam(std::string ARMOR_TYPE)
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
void EKF::setDebugLogger(bool SHOW_LOGGER_DEBUG)
{
    this->SHOW_LOGGER_DEBUG = SHOW_LOGGER_DEBUG;
}



void EKF::reset()
{
    this->is_initialized = false; // 重置初始化
}


// 05【整车中心坐标系】-> 四个【装甲板坐标系】
void EKF::updateCarCenterToArmorplate(std::string child_frame, double x, double y, double z, double roll, double pitch, double yaw)
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


void EKF::updateFourArmorplates()
{
    // 前装甲板： (R, 0, 0)，x轴朝向正前方（yaw=π）
    updateCarCenterToArmorplate("front_armorplate", this->radius, 0.0, 0.0, 0.0, 0.0, M_PI);
    
    // 后装甲板： (-R, 0, 0)，x轴朝向正后方（yaw=0）这与整车中心坐标系的 姿态 相等
    updateCarCenterToArmorplate("behind_armorplate", -this->radius, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    // 左装甲板： (0, R, 0)，x轴朝向左方（yaw=-π/2）
    updateCarCenterToArmorplate("left_armorplate", 0.0, this->radius, 0.0, 0.0, 0.0, -M_PI/2);
    
    // 右装甲板： (0, -R, 0)，x轴朝向右方（yaw=π/2）
    updateCarCenterToArmorplate("right_armorplate", 0.0, -this->radius, 0.0, 0.0, 0.0, M_PI/2);
}


// 05 【世界坐标系】->【整车中心坐标系】注意这里忽略了 pitch & roll
void EKF::updateWorldToCarCenter()
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->node_->now(); // 帧头 -> 直接获取当前时刻
    tf.header.frame_id = "camera_frame"; // 父坐标系 -> 世界坐标系 // 没有电控调试时，把坐标系名字改为 camera_frame
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



// // 查询【世界坐标系】-> 【某个装甲板坐标系】和中心点在世界下坐标
// bool EKF::getTransform(double& x_predict, double& y_predict, double& z_predict, double& center)
// {
//     geometry_msgs::msg::TransformStamped transform_world_armorplate; // 世界 -> 整车中心
//     try 
//     {
//         transform_world_armorplate = tf_buffer_->lookupTransform("world_frame", "car_center_frame", tf2::TimePointZero);
//     } 
//     catch (tf2::TransformException &ex) 
//     {
//         RCLCPP_ERROR(node_->get_logger(), "【世界坐标系 -> 整车中心 坐标系】TF lookup failed: %s", ex.what());
//         return false; 
//     }

//     // 获得 世界 -> 装甲板 的平移向量 
//     x_c = transform_world_armorplate.transform.translation.x;
//     y_c = transform_world_armorplate.transform.translation.y;
//     z_c = transform_world_armorplate.transform.translation.z;

//     // 获得 世界 -> 装甲板 的旋转向量（四元数转欧拉角，取 yaw）
//     tf2::Quaternion q(
//         transform_world_armorplate.transform.rotation.x,
//         transform_world_armorplate.transform.rotation.y,
//         transform_world_armorplate.transform.rotation.z,
//         transform_world_armorplate.transform.rotation.w
//     );
//     double roll_c, pitch_c, yaw_c;
//     tf2::Matrix3x3(q).getRPY(roll_c, pitch_c, yaw_c); // 顺序：roll, pitch, yaw （XYZ）
    
//     yaw = yaw_c; // 返回的 yaw 是弧度

//     return true;
// }


// 根据 ID 获取装甲板在 整车中心坐标系 下的参数
void EKF::getArmorParams(double& dx, double& dy, double& theta_offset)
{
    switch(this->armor_id)
    {
        case 1: // 正前方装甲板 (+, 0)
        {
            dx = this->radius;
            dy = 0.0; 
            theta_offset = M_PI; // 180
            break;
        }
            

        case 2: // 右侧装甲板 (0, +)
        {
            dx = 0.0; 
            dy = -this->radius; 
            theta_offset = M_PI / 2; // +90  
            break;
        }

            
        case 3: // 正后方装甲板 (与中心坐标系同向) (-, 0)
        {
            dx = -this->radius; 
            dy = 0.0; 
            theta_offset = 0.0; // 0
            break;
        }
            
        
        case 4: // 左侧装甲板 (0, +)
        {
            dx = 0.0; 
            dy = this->radius; 
            theta_offset = -M_PI / 2; // -90  
            break;
        }
            
    }
}


// 这是滤波的实时值，而不是预测位置喵
// 得到某个 id 装甲板四个角点在世界下的坐标
void EKF::getArmorFourCorners(std::vector<Eigen::Vector3d>& corners, int armor_id)
{
    corners.resize(4);
    double x_c = this->X(0), y_c = this->X(1), z_c = this->X(2), yaw = this->X(6);
    this->armor_id = armor_id;
    double dx, dy, theta_offset;
    getArmorParams(dx, dy, theta_offset);

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


// 非线性观测方程 
Eigen::Matrix<double, 4, 1> EKF::h(const Eigen::Matrix<double, 9, 1>& X_in)
{
    double x_c = X_in(0), y_c = X_in(1), z_c = X_in(2);
    double yaw = X_in(6);
    double dx, dy, theta_offset;
    getArmorParams(dx, dy, theta_offset);

    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    // 装甲板坐标系 在整车中心坐标系下的坐标

    Eigen::Matrix<double, 4, 1> z_pred;
    z_pred(0) = x_c + cos_yaw * dx - sin_yaw * dy;   // x_armor
    z_pred(1) = y_c + sin_yaw * dx + cos_yaw * dy;   // y_armor
    z_pred(2) = z_c;                                 // z 相同
    z_pred(3) = yaw + theta_offset;                  // yaw_armor

    return z_pred;
}


// 观测矩阵 雅可比矩阵 H = ∂h/∂x
Eigen::Matrix<double, 4, 9> EKF::computeH(const Eigen::Matrix<double, 9, 1>& X_in)
{
    double yaw = X_in(6);
    double dx, dy, theta_offset;
    getArmorParams(dx, dy, theta_offset);  // theta_offset 用不到，只关心 dx, dy

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



// 发布整车的 tf 动态变换，拿到整车中心作为观测数据，进行预测
void EKF::getKalman(Eigen::Vector3d armorplate_center, double yaw_armor, int armor_id, double dt)
{
    this->armor_id = armor_id; // 传入的装甲板 id （1:前，2:右，3:后，4:左）

	// 如果没有初始化就初始化 
    if (this->is_initialized == false)
	{
        updateParamsFromServer();  // 确保读取有效值

		Initialized();
        
        // 根据第一次观测反算一个粗略的中心初始状态
        double dx, dy, theta_offset;
        getArmorParams(dx, dy, theta_offset);
        double init_yaw = yaw_armor - theta_offset;   // 粗略的中心偏航角

        // 反推中心位置
        double cos_yaw = cos(init_yaw), sin_yaw = sin(init_yaw);
        double init_x = armorplate_center[0] - (cos_yaw * dx - sin_yaw * dy);
        double init_y = armorplate_center[1] - (sin_yaw * dx + cos_yaw * dy);
        double init_z = armorplate_center[2];

        X_prev << init_x, init_y, init_z, 0.0, 0.0, 0.0, init_yaw, 0.0, 0.0;
        X = X_prev;
    
        return; // 第一帧不输出结果
	}

	// 根据dt更新参数
	CalculateParameter(dt); 

    Z << armorplate_center[0], armorplate_center[1], armorplate_center[2], yaw_armor; // 写入原始观测值

	///////////////////////////////////// 卡尔曼五步 //////////////////////////////////////

	StatusPredict(); // 状态预测

	UncertaintyPredict(); // 不确定性预测

    // 计算当前原始观测值的 雅可比矩阵（偏导数矩阵），在 X_est 处求值
    H = computeH(X_est);

	CalculateKalmanGain(); // 计算卡尔曼增益

	
	UpdateStatus(); // 用测量值更新状态

	UpdateUncertainty(); // 更新不确定性

    // 偏航角归一化到 [-π, π]
    X(6) = std::atan2(std::sin(X(6)), std::cos(X(6)));

	UpdateHistoricalData(); // 更新历史数据

    updateFourArmorplates(); // 发布 整车中心 -> 四个装甲板 的 tf 坐标系变换

    updateWorldToCarCenter(); // 发布 世界 -> 整车中心 的 tf 坐标系
}



// 初始化  状态转移矩阵F  协方差矩阵P  预测过程噪声Q  观测矩阵H  测量过程噪声R
void EKF::Initialized()
{

	// 协方差矩阵
	P_prev = 0.1 * I;


	// 测量过程噪声 x y z yaw
    // 单位m rad
	R << this->r_x_, 0, 0, 0,
        0, this->r_y_, 0, 0,
        0, 0, this->r_z_, 0,
        0, 0, 0, this->r_yaw_;

	this->is_initialized = true;
}


// 依据 dt 更新参数
void EKF::CalculateParameter(double dt)
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
    R << this->r_x_, 0, 0, 0,
        0, this->r_y_, 0, 0,
        0, 0, this->r_z_, 0,
        0, 0, 0, this->r_yaw_;

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
    Eigen::Matrix<double, 4, 1> innovation = Z - h(X_est);
    
    // 将角度差 (yaw偏差) 限制在 [-π, π] 的最短路径上！
    // 否则比如 从 +179 到 -179，就会跳 358 度！但是用 sin 和 cos 就可以归一化到 -PI 到 PI 内，再 atan2 反求角度
    innovation(3) = std::atan2(std::sin(innovation(3)), std::cos(innovation(3)));


    // 3. 【新增】角度跳变门控防爆机制
    // 如果观测角度与预测角度偏差大于 35度 (约 0.6 rad)，说明 PnP 肯定算错了
    // 只要滤波器已经初始化，我们绝对不相信这个离谱的观测，强行将其拉回 0
    if (std::abs(innovation(3)) > 0.6 && this->is_initialized) 
    {
        innovation(3) = 0.0; // 离谱的角度观测我不听，用 EKF 自己的预测！
    }

    // 再用处理后的新息进行状态更新
    X = X_est + K * innovation;

    // 再归一化
    X(6) = std::atan2(std::sin(X(6)), std::cos(X(6))); 

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
	P = (I - K * H) * P_est;
}


// 更新历史数据
void EKF::UpdateHistoricalData()
{
	X_prev = X;
	P_prev = P;
}



// 预测未来 future 秒的中心点的位置，通过传入引用，获得 x y z
void EKF::getCenterPredict(Eigen::Vector3d& car_center_predict, double future_time)
{
	car_center_predict[0] = this->X(0) + this->X(3) * future_time; // x + v_x * future_time
    car_center_predict[1] = this->X(1) + this->X(4) * future_time; // y + v_y * future_time
    car_center_predict[2] = this->X(2) + this->X(5) * future_time; // z + v_z * future_time
}

// 获得装甲板的预测位置
void EKF::getArmorPredict(Eigen::Vector3d& armorplate_center_predict, 
                          int armor_id, double future_time) 
{
    // 1. 先预测未来中心状态
    double future_x_c = this->X(0) + this->X(3) * future_time; // x + v_x * future_time
    double future_y_c = this->X(1) + this->X(4) * future_time; // y + v_y * future_time
    double future_z_c = this->X(2) + this->X(5) * future_time; // z + v_z * future_time
    double future_yaw = X(6) + X(7) * future_time + 0.5 * X(8) * future_time * future_time;

    // 2. 根据装甲板ID获取几何参数
    this->armor_id = armor_id;
    double dx, dy, theta_offset;
    getArmorParams(dx, dy, theta_offset);

    // 3. 应用旋转得到装甲板世界坐标
    double cos_yaw = cos(future_yaw);
    double sin_yaw = sin(future_yaw);

    armorplate_center_predict[0] = future_x_c + cos_yaw * dx - sin_yaw * dy;
    armorplate_center_predict[1] = future_y_c + sin_yaw * dx + cos_yaw * dy;
    armorplate_center_predict[2] = future_z_c;
}


// 改变滤波器内部状态的预测，会更新状态
void EKF::predictOnly(double dt)
{
    if (!this->is_initialized) return;

    CalculateParameter(dt);
    StatusPredict();
    UncertaintyPredict();

    X = X_est; // 将预测状态作为当前状态
    P = P_est;

    X(6) = std::atan2(std::sin(X(6)), std::cos(X(6)));

    X_prev = X;
    P_prev = P;
}

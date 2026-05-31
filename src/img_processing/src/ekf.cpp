#include <ekf.hpp>


EKF::EKF(rclcpp::Node* node): node_(node)
{
	this->is_initialized = false; // 没有初始化

    car_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
	
    // 创建 TF 缓存和监听器（用来查询【世界坐标系 -> 整车中心坐标系】是否可以变换）
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    I = Eigen::Matrix<double, 11, 11>::Identity(); // 9*9 单位矩阵
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
    
    this->q_r_ = node_->get_parameter("ekf.q_r").as_double();
    this->q_dz_ = node_->get_parameter("ekf.q_dz").as_double();

    // 观测矩阵 r
    this->r_los_yaw_ = node_->get_parameter("ekf.r_los_yaw").as_double();
    this->r_los_pitch_ = node_->get_parameter("ekf.r_los_pitch").as_double();
    this->r_distance_ = node_->get_parameter("ekf.r_distance").as_double();
    this->r_euler_yaw_ = node_->get_parameter("ekf.r_euler_yaw").as_double();

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
        double dx, dy, dz_offset, theta_offset;
        GetArmorplateParams(this->X, i, dx, dy, dz_offset, theta_offset);
        
        std::string frame_name = "armor_" + std::to_string(i);
        UpdateCarCenterToArmorplate(frame_name, dx, dy, dz_offset, 0.0, 15.0 * M_PI / 180.0, theta_offset);
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
void EKF::GetArmorplateParams(const Eigen::Matrix<double, 11, 1>& X_in, int id, double& dx, double& dy, double& dz_offset, double& theta_offset)
{
    // 严格按照 0123 体系：偏差角 = ID * (360 / 装甲板数量)
    theta_offset = id * 2.0 * M_PI / armor_num_;
    NormalizeAngle(theta_offset);

    // 四块装甲板的情况下，动态提取：0和2号板取 r1 (X8)，1和3号板取 r2 (X9)
    double truly_radius = X_in(8);
    dz_offset = 0;

    if(armor_num_ == 4)
    {
        if (id == 0 || id == 2)
        {
            truly_radius = X_in(8);
            dz_offset = 0; // 0、2 号板没有高低板偏移，就等于车体中心高度 X(2)
        }
        else 
        {
            truly_radius = X_in(9);
            dz_offset = X_in(10); 
        }
    }
    

    // 装甲板的 +X 轴指向车心，所以它处于车心的反方向
    dx = -truly_radius * std::cos(theta_offset);
    dy = -truly_radius * std::sin(theta_offset);
}


// 这是滤波的实时值，而不是预测位置喵
// 得到某个 id 装甲板四个角点在世界下的坐标
void EKF::GetArmorplateFourCorners(std::vector<Eigen::Vector3d>& corners, int armor_id)
{
    corners.resize(4);
    double x_c = this->X(0), y_c = this->X(1), z_c = this->X(2), yaw = this->X(6);
    this->armor_id = armor_id;
    
    double dx, dy, dz_offset, theta_offset;
    GetArmorplateParams(this->X, armor_id, dx, dy, dz_offset, theta_offset); // 根据装甲板 ID 和当前状态 X 获取装甲板在车体坐标系下的偏移参数

    // 算出装甲板物理中心 (必须乘上车体的 Yaw 旋转！)

    Eigen::AngleAxisd car_yaw_rot(yaw, Eigen::Vector3d::UnitZ()); // 车体旋转矩阵
    Eigen::Vector3d offset_local(dx, dy, dz_offset); // 静止下装甲板中心相对于车心的偏移（在车体坐标系下），加入了高低板的补偿
    
    // 装甲板世界绝对中心 = 整车世界绝对中心 + (车体旋转矩阵 × 装甲板相对车心的偏移)
    Eigen::Vector3d car_center = Eigen::Vector3d(x_c, y_c, z_c);
    Eigen::Vector3d armor_center = car_center + car_yaw_rot * offset_local;

    // =========================================================
    // 2. 装甲板绝对姿态：车体Yaw + 相对偏移角
    // =========================================================
    double armor_yaw = yaw + theta_offset;
    
    // 【完美契合你的右手系法则】：15度物理倾角
    double armor_pitch = 15.0 * M_PI / 180.0; 

    // 构建装甲板在世界坐标系下的绝对旋转矩阵 R (先 Pitch 后 Yaw)
    Eigen::AngleAxisd yawAngle(armor_yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(armor_pitch, Eigen::Vector3d::UnitY());
    Eigen::Matrix3d R_armor = (yawAngle * pitchAngle).toRotationMatrix();

    // =========================================================
    // 3. 定义本地四角点坐标 
    // 严格按照 armorplate.cpp 的定义顺序：左上、左下、右下、右上
    // 在装甲板本地系中，+X向内(车心)，+Y向左，+Z向上
    // =========================================================
    double half_w = this->width / 2.0;
    double half_h = this->height / 2.0;

    std::vector<Eigen::Vector3d> local_corners = 
    {
        Eigen::Vector3d(0.0,  half_w,  half_h), // 左上
        Eigen::Vector3d(0.0,  half_w, -half_h), // 左下
        Eigen::Vector3d(0.0, -half_w, -half_h), // 右下
        Eigen::Vector3d(0.0, -half_w,  half_h)  // 右上
    };

    // =========================================================
    // 4. 将本地点映射到世界系
    // =========================================================
    for (int i = 0; i < 4; ++i) 
    {
        // 角点世界坐标 = 装甲板世界绝对中心 + (装甲板绝对旋转矩阵 × 角点相对装甲板中心的偏移)
        corners[i] = armor_center + R_armor * local_corners[i];
    }
}


// 非线性观测方程 h(x, armor_id)：将 整车预测的状态量XYZ 映射到 装甲板观测空间XYZ
Eigen::Matrix<double, 4, 1> EKF::h(const Eigen::Matrix<double, 11, 1>& X_in)
{
    double x_c = X_in(0), y_c = X_in(1), z_c = X_in(2);
    double yaw = X_in(6);
    double dx, dy, dz_offset, theta_offset;
    GetArmorplateParams(X_in, this->armor_id, dx, dy, dz_offset, theta_offset);

    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    // 装甲板坐标系 在整车中心坐标系下的坐标

    Eigen::Matrix<double, 4, 1> z_pred;
    z_pred(0) = x_c + cos_yaw * dx - sin_yaw * dy;   // x_armor
    z_pred(1) = y_c + sin_yaw * dx + cos_yaw * dy;   // y_armor
    z_pred(2) = z_c + dz_offset;                     // z 注意存在高度差（1、3号装甲板高度 - 0、2号装甲板高度也就是车体中心高度 = dz_offset）！
    z_pred(3) = yaw + theta_offset;                  // yaw_armor
    NormalizeAngle(z_pred(3)); // 归一化装甲板的 yaw 到 [-pi, pi]

    return z_pred;
}


// 观测矩阵 雅可比矩阵 H = ∂h/∂x
Eigen::Matrix<double, 4, 11> EKF::ComputeH(const Eigen::Matrix<double, 11, 1>& X_in)
{
    double yaw = X_in(6);
    double dx, dy, dz_offset, theta_offset;
    GetArmorplateParams(X_in, this->armor_id, dx, dy, dz_offset, theta_offset);

    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    Eigen::Matrix<double, 4, 11> H_mat = Eigen::Matrix<double, 4, 11>::Zero();

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

    // 对装甲板相对偏移量的偏导（r1, r2, dz）
    // 四块装甲板的时候会用到高度差 dz，才可能用到 r2
    if (armor_num_ == 4 && (this->armor_id == 1 || this->armor_id == 3)) 
    {
        // 使用 r2，对 r2 (X9) 求偏导
        H_mat(0, 9) = -std::cos(yaw + theta_offset);
        H_mat(1, 9) = -std::sin(yaw + theta_offset);
        // 对 dz (X10) 求偏导：z_pred = z_c + dz
        H_mat(2, 10) = 1.0;
    } 
    else 
    {
        // 使用 r1，对 r1 (X8) 求偏导
        H_mat(0, 8) = -std::cos(yaw + theta_offset);
        H_mat(1, 8) = -std::sin(yaw + theta_offset);
    }

    // 其余偏导（对速度、角速度）均为 0
    return H_mat;
}



///////////////////////////// YPD 相关函数的实现 ///////////////////////////////////

// 【新增】：将状态量直接映射为 YPD 观测量的非线性函数
// 用当前预测的整车状态 X_est​ (XYZ) 反推出来的预测值 YPD
Eigen::Matrix<double, 4, 1> EKF::h_ypd(const Eigen::Matrix<double, 11, 1>& X_in)
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


// 【新增】：利用链式求导法则，算出 YPD 坐标系下的 4x11 雅可比矩阵
Eigen::Matrix<double, 4, 11> EKF::ComputeH_YPD(const Eigen::Matrix<double, 11, 1>& X_in)
{
    // 1. 先复用你原来写好的 XYZ 雅可比矩阵 H_xyz (4x11)
    Eigen::Matrix<double, 4, 11> H_xyz = this->ComputeH(X_in);

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
        J_ypd(0, 0) = -y / r2,            J_ypd(0, 1) =  x / r2,            J_ypd(0, 2) =  0.0;

        J_ypd(1, 0) = (z * x) / (d2 * r), J_ypd(1, 1) = (z * y) / (d2 * r), J_ypd(1, 2) = -r / d2;

        J_ypd(2, 0) = x / d,              J_ypd(2, 1) = y / d,              J_ypd(2, 2) = z / d;
    }

    // 4. 链式法则魔法：H_new_top3 = J_ypd * H_xyz_top3
    Eigen::Matrix<double, 4, 11> H_new = Eigen::Matrix<double, 4, 11>::Zero();
    H_new.block<3, 11>(0, 0) = J_ypd * H_xyz.block<3, 11>(0, 0); 
    H_new.row(3) = H_xyz.row(3); // 第 4 行 (装甲板yaw的导数) 保持不变

    return H_new;
}


void EKF::NormalizeAngle(double& angle)
{
    angle = std::atan2(std::sin(angle), std::cos(angle));
}



// 发布整车的 tf 动态变换，拿到整车中心作为观测数据，进行预测
void EKF::UpdateExtendedKalman(Eigen::Vector3d armorplate_center, double armor_yaw_origin, int armor_id, double dt)
{
    this->armor_id = armor_id; // 传入的装甲板 id （1:前，2:右，3:后，4:左）

	// 如果没有初始化就初始化 
    if (this->is_initialized == false)
	{
        UpdateParamsFromServer();  // 确保读取有效值

		Initialized(armorplate_center, armor_yaw_origin);
        
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

    // 同时更新 R 矩阵（因为参数可能已改变）

    double center_yaw = std::atan2(obs_y, obs_x);
    double delta_angle = armor_yaw_origin - center_yaw;
    NormalizeAngle(delta_angle); // 偏角越大，越不准

    double dynamic_r_dist = std::log(std::abs(delta_angle) + 1.0) + r_distance_;
    double dynamic_r_yaw = std::log(std::abs(obs_d) + 1.0) / 200.0 + r_euler_yaw_;


    R << r_los_yaw_, 0, 0, 0,
        0, r_los_pitch_, 0, 0,
        0, 0, dynamic_r_dist, 0,
        0, 0, 0, dynamic_r_yaw;

    // 顺序必须是：[yaw_cam, pitch_cam, distance, armor_yaw_origin]
    Z << obs_los_yaw, obs_los_pitch, obs_d, armor_yaw_origin;

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
    // 赋予未知的几何尺寸极大的不确定性，让它瞬间通过观测收敛！
    P_prev(8, 8) = 0.01; // r1
    P_prev(9, 9) = 0.01; // r2
    P_prev(10, 10) = 0.01; // dz

    // 换成 YPD 球面坐标系 
	// 测量过程噪声矩阵
    // 单位m rad
	R << r_los_yaw_, 0, 0, 0,
        0, r_pitch, 0, 0,
        0, 0, r_distance_, 0,
        0, 0, 0, r_euler_yaw_;

    // 根据第一次观测反算一个粗略的中心初始状态
    // 假设 r1 r2 初始都是 0.25 米，dz 初始都是 0.05 米（1、3号装甲板相对于0、2号装甲板，高度-0.05m）
    double init_r1 = 0.25;
    double init_r2 = 0.25;
    double init_dz = 0.00;
    Eigen::Matrix<double, 11, 1> tmp_X = Eigen::Matrix<double, 11, 1>::Zero();
    tmp_X(8) = init_r1, tmp_X(9) = init_r2, tmp_X(10) = init_dz;

    double dx, dy, dz_offset, theta_offset;
    GetArmorplateParams(tmp_X, this->armor_id, dx, dy, dz_offset, theta_offset);

    double init_yaw = yaw_armor - theta_offset;  // 根据装甲板的观测 yaw 和 预设的偏移角，反推整车的初始 yaw
    NormalizeAngle(init_yaw);

    // 反推中心位置
    double cos_yaw = cos(init_yaw), sin_yaw = sin(init_yaw);
    double init_x = armorplate_center[0] - (cos_yaw * dx - sin_yaw * dy);
    double init_y = armorplate_center[1] - (sin_yaw * dx + cos_yaw * dy);
    double init_z = armorplate_center[2] - dz_offset; // 注意存在高度差（1、3号装甲板高度 - 0、2号装甲板高度也就是车体中心高度 = dz_offset）！
                                                      // 因为要反推车体中心高度，所以就是这个写法

    X_prev << init_x, init_y, init_z, 0.0, 0.0, 0.0, init_yaw, 0.0, init_r1, init_r2, init_dz;
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

    /*
        状态转移矩阵 F
        x_k = x_k-1 + v_x * dt
        y_k = y_k-1 + v_y * dt
        z_k = z_k-1 + v_z * dt
        v_x_k = v_x_k-1 
        v_y_k = v_y_k-1 
        v_z_k = v_z_k-1
        yaw_k = yaw_k-1 + omega_k-1 * dt
        omega_k = omega_k-1
        r1 = r1
        r2 = r2
        dz = dz
    */
	F << 1, 0, 0, dt, 0,  0,  0, 0,  0, 0, 0,
         0, 1, 0, 0,  dt, 0,  0, 0,  0, 0, 0,
         0, 0, 1, 0,  0,  dt, 0, 0,  0, 0, 0,
         0, 0, 0, 1,  0,  0,  0, 0,  0, 0, 0,
         0, 0, 0, 0,  1,  0,  0, 0,  0, 0, 0,
         0, 0, 0, 0,  0,  1,  0, 0,  0, 0, 0,
         0, 0, 0, 0,  0,  0,  1, dt, 0, 0, 0,
         0, 0, 0, 0,  0,  0,  0, 1,  0, 0, 0,
         0, 0, 0, 0,  0,  0,  0, 0,  1, 0, 0,
         0, 0, 0, 0,  0,  0,  0, 0,  0, 1, 0,
         0, 0, 0, 0,  0,  0,  0, 0,  0, 0, 1;


	// 预测过程噪声矩阵 Q
	// Q << q_x_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //      0, q_y_, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //      0, 0, q_z_, 0, 0, 0, 0, 0, 0, 0, 0,
    //      0, 0, 0, q_v_x_, 0, 0, 0, 0, 0, 0, 0,
    //      0, 0, 0, 0, q_v_y_, 0, 0, 0, 0, 0, 0,
    //      0, 0, 0, 0, 0, q_v_z_, 0, 0, 0, 0, 0,
    //      0, 0, 0, 0, 0, 0, q_yaw_, 0, 0, 0, 0,
    //      0, 0, 0, 0, 0, 0, 0, q_omega_, 0, 0, 0,
    //      0, 0, 0, 0, 0, 0, 0, 0, q_r_, 0, 0,
    //      0, 0, 0, 0, 0, 0, 0, 0, 0, q_r_, 0,
    //      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, q_dz_;

    double v1 = 50.00;  // 加速度方差
    double v2 = 200.00;  // 角加速度方差
    double a = dt * dt * dt * dt / 4;
    double b = dt * dt * dt / 2;
    double c = dt * dt;
    
    Q << a*v1, 0, 0, b*v1, 0, 0, 0, 0, 0, 0, 0,
         0, a*v1, 0, 0, b*v1, 0, 0, 0, 0, 0, 0,
         0, 0, a*v1, 0, 0, b*v1, 0, 0, 0, 0, 0,
         b*v1, 0, 0, c*v1, 0, 0, 0, 0, 0, 0, 0,
         0, b*v1, 0, 0, c*v1, 0, 0, 0, 0, 0, 0,
         0, 0, b*v1, 0, 0, c*v1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, a*v2, b*v2, 0, 0, 0,
         0, 0, 0, 0, 0, 0, b*v2, c*v2, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

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
    
    // 将视线角 (los_yaw, los_pitch) 和目前姿态 (yaw_armor) 全部归一化！
    // 否则比如 从 +179 到 -179，就会跳 358 度！但是用 sin 和 cos 就可以归一化到 -PI 到 PI 内，再 atan2 反求角度
    NormalizeAngle(innovation(0)); // los_yaw 归一化
    NormalizeAngle(innovation(1)); // los_pitch 归一化
    NormalizeAngle(innovation(3)); // yaw_armor 归一化

    // 计算卡方值 (马氏距离的平方)
    Eigen::Matrix<double, 4, 4> S = H * P_est * H.transpose() + R;
    double nis = innovation.transpose() * S.inverse() * innovation;

    // 自由度为4，95%置信度下的卡方阈值约为 9.488
    if (nis > 9.488 && this->is_initialized) 
    {
        // 这是一个严重的异常点（假装甲板），拒绝更新状态！
        RCLCPP_ERROR(rclcpp::get_logger("ekf_debug"), "Outlier detected! NIS=%.2f exceeds threshold. Rejecting update.", nis);  
        X = X_est; 
        P = P_est;
        return; 
    }


    // 再用处理后的新息进行状态更新
    X = X_est + K * innovation;
    NormalizeAngle(X(6)); 

    if(this->SHOW_LOGGER_DEBUG)
    {
        // ---------- 临时调参日志 ----------
        RCLCPP_INFO(rclcpp::get_logger("ekf_debug"),
            "Innovation: los_yaw=%.4f, los_pitch=%.4f, los_distance=%.4f, yaw=%.4f",
            innovation(0), innovation(1), innovation(2), innovation(3));

        RCLCPP_INFO(rclcpp::get_logger("ekf_debug"), "State: x=%.3f, y=%.3f, z=%.3f, vx=%.3f, vy=%.3f, vz=%.3f, yaw=%.3f, omega=%.3f, r1=%.3f, r2=%.3f, dz=%.3f",
            X(0), X(1), X(2), X(3), X(4), X(5), X(6), X(7), X(8), X(9), X(10));

        RCLCPP_INFO(rclcpp::get_logger("ekf_debug"),
            "P_diag: px=%.4f, py=%.4f, pz=%.4f, pyaw=%.4f",
            P(0,0), P(1,1), P(2,2), P(6,6));

        RCLCPP_INFO(rclcpp::get_logger("ekf_debug"),
            "K_gain: Kx=%.4f, Ky=%.4f, Kz=%.4f, Kyaw=%.4f",
            K(0,2), K(1,0), K(2,1), K(6,3)); 
            // 现在的打印才是：x由距离控制，y由偏航控制，z由俯仰控制！
        // ---------- 日志结束 ----------
    }
    
}


// 更新协方差，不确定性 P_k = (I - K_k * H) * P_k_est
void EKF::UpdateUncertainty()
{
	// P = (I - K * H) * P_est;
    P = (I - K * H) * P_est * (I - K * H).transpose() + K * R * K.transpose(); // Joseph form 形式更新协方差矩阵，数值更稳定，防止 P 变成非正定矩阵
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
    double future_yaw = X(6) + X(7) * future_time;
    NormalizeAngle(future_yaw);

    // 2. 根据装甲板ID获取几何参数
    double dx, dy, dz_offset, theta_offset;
    GetArmorplateParams(this->X, armor_id, dx, dy, dz_offset, theta_offset);

    // 3. 应用旋转得到装甲板世界坐标
    double cos_yaw = cos(future_yaw);
    double sin_yaw = sin(future_yaw);

    armorplate_center_predict[0] = future_x_c + cos_yaw * dx - sin_yaw * dy;
    armorplate_center_predict[1] = future_y_c + sin_yaw * dx + cos_yaw * dy;
    armorplate_center_predict[2] = future_z_c + dz_offset; // 注意存在高度差（1、3号装甲板高度 - 0、2号装甲板高度也就是车体中心高度 = dz_offset）！
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
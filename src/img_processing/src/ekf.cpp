#include <ekf.hpp>


// ============================== 构造与重置初始化 ==============================

EKF::EKF(rclcpp::Node* node) : node_(node)
{
    this->is_initialized_ = false; 

    // 初始化 ROS2 TF 广播器与缓存、监听器（用来查询【父坐标系】->【整车中心坐标系】是否可以变换）
    car_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    tf_buffer_       = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_     = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 初始化 11x11 单位矩阵
    I = Eigen::Matrix<double, 11, 11>::Identity(); 
}

void EKF::Reset() 
{ 
    this->is_initialized_ = false; 
}


// ============================== 参数与配置接口 ==============================


// 从参数服务器，动态获得参数
void EKF::UpdateParamsFromServer() 
{
    if (!node_) return;

    // 预测矩阵 Q 参数（半废弃，暂时不采用）
    this->q_x_     = node_->get_parameter("ekf.q_x").as_double();
    this->q_y_     = node_->get_parameter("ekf.q_y").as_double();
    this->q_z_     = node_->get_parameter("ekf.q_z").as_double();
    this->q_v_x_   = node_->get_parameter("ekf.q_v_x").as_double();
    this->q_v_y_   = node_->get_parameter("ekf.q_v_y").as_double();
    this->q_v_z_   = node_->get_parameter("ekf.q_v_z").as_double();
    this->q_yaw_   = node_->get_parameter("ekf.q_yaw").as_double();
    this->q_omega_ = node_->get_parameter("ekf.q_omega").as_double();
    this->q_r_     = node_->get_parameter("ekf.q_r").as_double();
    this->q_dz_    = node_->get_parameter("ekf.q_dz").as_double();

    // 预测矩阵 Q 参数 白噪声模型
    this->q_v1_    = node_->get_parameter("ekf.q_v1").as_double(); // 平移加速度方差
    this->q_v2_    = node_->get_parameter("ekf.q_v2").as_double(); // 旋转角加速度方差

    // 观测矩阵 R 参数
    this->r_los_yaw_   = node_->get_parameter("ekf.r_los_yaw").as_double();
    this->r_los_pitch_ = node_->get_parameter("ekf.r_los_pitch").as_double();
    this->r_distance_  = node_->get_parameter("ekf.r_distance").as_double();
    this->r_euler_yaw_ = node_->get_parameter("ekf.r_euler_yaw").as_double();

    // 父坐标系与调试日志开关
    this->father_frame_ = node_->get_parameter("core.mode.is_standalone_mode").as_bool() ? "camera_frame" : "world_frame";
    this->SHOW_LOGGER_DEBUG = node_->get_parameter("ekf.show_logger_debug").as_bool();
}


// 设置装甲板的 width 和 height
// todo: 可以进一步扩展为直接从参数服务器读取装甲板尺寸，支持更多种类的装甲板
void EKF::SetArmorplateSize(std::string ARMOR_TYPE)
{
    if(ARMOR_TYPE == "normal") 
    {
        this->width_  = 0.135;
        this->height_ = 0.055;
    }
    else 
    {
        this->width_  = 0.225;
        this->height_ = 0.055;
    }
}


void EKF::SetArmorNum(int num) 
{ 
    this->armor_num_ = num; 
}


// ============================== 核心算法接口【流水线】 ==============================


// 1. 根据首次观测数据完成 EKF 的状态初始化
void EKF::Initialized(const Eigen::Vector3d& armorplate_center, double yaw_armor, int id, rclcpp::Time current_image_time)
{
    armor_id_ = id;
    current_image_time_ = current_image_time;
    UpdateParamsFromServer(); // 确保参数最新

    P.setIdentity();
    P(0,0) = 1.0; P(1,1) = 1.0; P(2,2) = 1.0;          // XYZ 位置
    P(3,3) = 64.0; P(4,4) = 64.0; P(5,5) = 64.0;       // XYZ 速度
    P(6,6) = 0.5;                                                // Yaw 角度
    P(7,7) = 100.0;                                              // 角速度 Omega

    P(8, 8)   = 1.0;   // r1
    P(9, 9)   = 1.0;   // r2 
    P(10, 10) = 1.0;   // dz

    /*
    R << r_los_yaw_,            0,           0,            0,
                  0, r_los_pitch_,           0,            0,
                  0,            0, r_distance_,            0,
                  0,            0,           0, r_euler_yaw_;
    */

    // 根据第一次观测反算一个粗略的中心初始状态
    // r1 r2 初始 0.25 m，dz 初始是 0.05 m（1、3号装甲板相对于0、2号装甲板的高度差，计当前 1、3号装甲板高度为车心高度，初始高度差 0m）
    double init_r1 = 0.25;
    double init_r2 = 0.25;
    double init_dz = 0.00;
    
    Eigen::Matrix<double, 11, 1> tmp_X = Eigen::Matrix<double, 11, 1>::Zero();
    tmp_X(8) = init_r1;
    tmp_X(9) = init_r2;
    tmp_X(10) = init_dz;

    double dx, dy, dz_offset, theta_offset;
    GetArmorplateParams(tmp_X, this->armor_id_, dx, dy, dz_offset, theta_offset);

    // 根据装甲板的观测 yaw 和 预设的当前板离整车角度（0板角度）的偏移角，反推整车中心 Yaw
    double init_yaw = yaw_armor - theta_offset;  
    NormalizeAngle(init_yaw);

    // 反推车体几何中心 XYZ
    double cos_yaw = cos(init_yaw), sin_yaw = sin(init_yaw);
    double init_x = armorplate_center[0] - (cos_yaw * dx - sin_yaw * dy);
    double init_y = armorplate_center[1] - (sin_yaw * dx + cos_yaw * dy);
    double init_z = armorplate_center[2] - dz_offset;

    X << init_x, init_y, init_z, 0.0, 0.0, 0.0, init_yaw, 0.0, init_r1, init_r2, init_dz;

    this->is_initialized_ = true;
}


// 2. 状态预测，将状态通过预测推演到当前时间
// 注意！一定要添传入 current_image_time 参数，否则 current_image_time 只在 Updatestate 函数才会更新，TF 发布时间不同步！
void EKF::PredictState(double dt, rclcpp::Time current_image_time)
{
    if (!this->is_initialized_) return;

    current_image_time_ = current_image_time; // 确保滤波器时间最新！不要没看到装甲板，滤波器时间就不更新！

    UpdateParamsFromServer(); // 动态调参实时生效
    UpdateParameters(dt);     // 刷新 F, Q

    X = F * X;                         // [1] 状态预测
    P = F * P * F.transpose() + Q;     // [2] 协方差预测

    NormalizeAngle(X(6));

    // 盲推 / 预测阶段，也要发布 TF 保证系统连续性
    UpdateFatherToCarCenter();
    UpdateCarCenterToArmorplates(); 
}


// 3. 状态更新，将状态通过观测数据更新
void EKF::UpdateState(const Eigen::Vector3d& armorplate_center, double euler_yaw, int id, rclcpp::Time current_image_time)
{
    armor_id_ = id;                             // 传入装甲板 ID (0:后, 1:右, 2:前, 3:左)
    current_image_time_ = current_image_time;   // 更新当前图像时间戳，用于后续 TF 发布的时间同步，非常重要


    // 将 XYZ 笛卡尔系下的观测，转化为 YPD 球坐标系下的观测
    double obs_x = armorplate_center[0];
    double obs_y = armorplate_center[1];
    double obs_z = armorplate_center[2];

    double obs_los_yaw   = std::atan2(obs_y, obs_x);
    double obs_los_pitch = -std::atan2(obs_z, std::sqrt(obs_x * obs_x + obs_y * obs_y));
    double obs_d         = std::sqrt(obs_x * obs_x + obs_y * obs_y + obs_z * obs_z);

    // 计算动态自适应 R 矩阵
    double delta_angle = euler_yaw - obs_los_yaw;
    NormalizeAngle(delta_angle); 

    // 偏角越大，深度距离测得越不准；距离越远，姿态偏角测得越不准 (对数衰减)
    double dynamic_r_dist = std::log(std::abs(delta_angle) + 1.0) + r_distance_;
    double dynamic_r_yaw  = std::log(std::abs(obs_d) + 1.0) / 200.0 + r_euler_yaw_;

    // 限制最大方差，防止滤波器彻底摆烂
    dynamic_r_dist = std::min(dynamic_r_dist, 10.0); // sqrt(100) = 10 m 的距离误差已经非常离谱了
    dynamic_r_yaw  = std::min(dynamic_r_yaw, 0.5); // sqrt(0.5) = 0.707 rad 的角度误差已经非常离谱了

    R << r_los_yaw_,            0,              0,              0,
                  0, r_los_pitch_,              0,              0,
                  0,            0, dynamic_r_dist,              0,
                  0,            0,              0,  dynamic_r_yaw;

    // 组合最终 YPD 观测向量 [los_yaw, los_pitch, distance, armor_yaw]
    Z << obs_los_yaw, obs_los_pitch, obs_d, euler_yaw;

    // 计算当前预测工作点下的雅可比矩阵，即先计算当前原始观测值的 雅可比矩阵（偏导数矩阵）在 X 处求值，再转换到 YPD 球面坐标系下
    H = ComputeH_YPD(X); 

    // 计算 YPD系下 的观测新息 Innovation
    /*
        h_ypd(X) 是用你当前预测的整车状态 X​ (XYZ) 反推出来的预测值 YPD (视线角和距离)
        而 Z 是你实际测量到的 YPD 观测值
        所以两者都是在 YPD 坐标系下的量，直接相减就得到新息
    */
    Eigen::Matrix<double, 4, 1> innovation = Z - h_ypd(X);
    NormalizeAngle(innovation(0)); // los_yaw
    NormalizeAngle(innovation(1)); // los_pitch
    NormalizeAngle(innovation(3)); // yaw_armor

    // NIS 卡方检验 -> 计算观测马氏距离，若极大，说明此帧观测严重背离系统先验，可能是异常数据
    Eigen::Matrix<double, 4, 4> S = H * P * H.transpose() + R;
    double nis = innovation.transpose() * S.inverse() * innovation;

    // 根据 nis 判断该帧数据是否有效，记入队列中（统计最近 10 帧）
    bool nis_failure = (nis > nis_threshold_) ? 1 : 0;
    recent_nis_failures_.push_back(nis_failure);
    if (recent_nis_failures_.size() > window_size_) 
    {
        recent_nis_failures_.pop_front(); // 保持滑动窗口大小
    }
    if (nis_failure)
    {
        RCLCPP_WARN(rclcpp::get_logger("ekf"), "[NIS FAILED] NIS: %f > %f", nis, nis_threshold_);
    }

    // 进行状态更新，必须严格遵循 EKF 状态更新流程，因为原本是有下标的！
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();          // [3] 卡尔曼增益
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();  // [4] 后验协方差更新
    X = X + K * innovation;                                                 // [5] 后验状态更新
    
    NormalizeAngle(X(6)); 

    // // === 新增：同济同款 NEES (状态跃变) 保护 ===
    // Eigen::Matrix<double, 11, 1> state_diff = X - X_prior;
    // NormalizeAngle(state_diff(6)); // 如果包含偏航角，做一下角度截断
    
    // 计算状态跃变马氏距离
    // double nees = state_diff.transpose() * P.inverse() * state_diff;


    // 打印 debug 日志
    if(this->SHOW_LOGGER_DEBUG) PrintDebugInfo(innovation);

    // 同步发布可视化与下发 TF 坐标变换
    UpdateFatherToCarCenter(); 
    UpdateCarCenterToArmorplates(); 
}


// ============================== 状态提取接口 ==============================

void EKF::GetCarCenterPredict(Eigen::Vector3d& car_center_predict, double future_time)
{
    car_center_predict[0] = this->X(0) + this->X(3) * future_time; 
    car_center_predict[1] = this->X(1) + this->X(4) * future_time; 
    car_center_predict[2] = this->X(2) + this->X(5) * future_time; 
}

void EKF::GetArmorplatePredict(Eigen::Vector3d& armorplate_center_predict, int armor_id, double future_time) 
{
    // 推演未来时刻的车心状态
    double future_x_c = this->X(0) + this->X(3) * future_time; 
    double future_y_c = this->X(1) + this->X(4) * future_time; 
    double future_z_c = this->X(2) + this->X(5) * future_time; 
    double future_yaw = X(6) + X(7) * future_time;
    NormalizeAngle(future_yaw);

    // 拉取理论几何偏距
    double dx, dy, dz_offset, theta_offset;
    GetArmorplateParams(this->X, armor_id, dx, dy, dz_offset, theta_offset);

    double cos_yaw = cos(future_yaw), sin_yaw = sin(future_yaw);

    armorplate_center_predict[0] = future_x_c + cos_yaw * dx - sin_yaw * dy;
    armorplate_center_predict[1] = future_y_c + sin_yaw * dx + cos_yaw * dy;
    armorplate_center_predict[2] = future_z_c + dz_offset; 
}


// 得到某个 id 装甲板四个角点现在（滤波后）在世界下的坐标
void EKF::GetArmorplateFourCorners(std::vector<Eigen::Vector3d>& corners, int armor_id)
{
    corners.resize(4);
    double x_c = this->X(0), y_c = this->X(1), z_c = this->X(2), yaw = this->X(6);
    this->armor_id_ = armor_id;
    
    double dx, dy, dz_offset, theta_offset;
    GetArmorplateParams(this->X, armor_id, dx, dy, dz_offset, theta_offset); // 根据装甲板 ID 和当前状态 X 获取某个 ID装甲板 在车体坐标系下的偏移参数

    Eigen::AngleAxisd car_yaw_rot(yaw, Eigen::Vector3d::UnitZ());  // 车体旋转矩阵
    Eigen::Vector3d offset_local(dx, dy, dz_offset); // 静止下装甲板中心相对于车心的偏移（在车体坐标系下），加入了高低板的补偿
    
    // 装甲板世界绝对中心 = 整车世界绝对中心 + (车体旋转矩阵 × 装甲板相对车心的偏移)
    Eigen::Vector3d car_center(x_c, y_c, z_c);
    Eigen::Vector3d armor_center = car_center + car_yaw_rot * offset_local;

    // 装甲板绝对姿态与仰角补偿 (装甲板前倾 15 度)
    double armor_yaw = yaw + theta_offset;
    double armor_pitch = 15.0 * M_PI / 180.0; 

    // 构建装甲板在世界坐标系下的绝对旋转矩阵 R (先 Pitch 后 Yaw)
    Eigen::AngleAxisd yawAngle(armor_yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(armor_pitch, Eigen::Vector3d::UnitY());
    Eigen::Matrix3d R_armor = (yawAngle * pitchAngle).toRotationMatrix();

    // 定义装甲板在装甲板坐标系的四角点坐标 
    // 严格按照 armorplate.cpp 的定义顺序：左上、左下、右下、右上
    // 在装甲板本地系中，+X向内(车心)，+Y向左，+Z向上
    double half_w = this->width_ / 2.0;
    double half_h = this->height_ / 2.0;

    std::vector<Eigen::Vector3d> local_corners = 
    {
        Eigen::Vector3d(0.0,  half_w,  half_h), // 左上
        Eigen::Vector3d(0.0,  half_w, -half_h), // 左下
        Eigen::Vector3d(0.0, -half_w, -half_h), // 右下
        Eigen::Vector3d(0.0, -half_w,  half_h)  // 右上
    };

    // 将装甲板坐标系下装甲板的四个角点，映射到世界系
    for (int i = 0; i < 4; ++i) 
    {
        // 角点世界坐标 = 装甲板世界绝对中心 + (装甲板绝对旋转矩阵 × 角点相对装甲板中心的偏移)
        corners[i] = armor_center + R_armor * local_corners[i];
    }
}


// ============================== 物理发散 与 NIS崩溃检测 接口 ==============================


// 物理发散检测，避免状态量太离谱
bool EKF::IsDiverged() const
{
    if (!is_initialized_) return false;
    if (X(8) < 0.10 || X(8) > 0.40 || X(9) < 0.10 || X(9) > 0.40 || std::abs(X(10)) > 0.15) return true;
    else return false;
}


// NIS 崩溃检测
bool EKF::IsNISFailed() const 
{
    if (recent_nis_failures_.size() < window_size_) return false;

    int total_failures = 0;
    for (int i = 0; i < recent_nis_failures_.size(); i++) 
    {
        total_failures += recent_nis_failures_[i];
    }
    
    bool is_nis_failed = (total_failures >= max_recent_nis_failures_);

    if (is_nis_failed) RCLCPP_ERROR(rclcpp::get_logger("ekf"), "[EKF::IsNISFailed()] NIS Failed! 在最近的 %d 帧中, NIS 失败了 %d 次, 阈值为 %d", window_size_, total_failures, max_recent_nis_failures_);
    return is_nis_failed;
}


void EKF::UpdateParameters(double dt)
{
    // 状态转移矩阵 F (严格运动学模型)
    F << 1, 0, 0, dt,  0,  0, 0,  0, 0, 0, 0,
         0, 1, 0,  0, dt,  0, 0,  0, 0, 0, 0,
         0, 0, 1,  0,  0, dt, 0,  0, 0, 0, 0,
         0, 0, 0,  1,  0,  0, 0,  0, 0, 0, 0,
         0, 0, 0,  0,  1,  0, 0,  0, 0, 0, 0,
         0, 0, 0,  0,  0,  1, 0,  0, 0, 0, 0,
         0, 0, 0,  0,  0,  0, 1, dt, 0, 0, 0,
         0, 0, 0,  0,  0,  0, 0,  1, 0, 0, 0,
         0, 0, 0,  0,  0,  0, 0,  0, 1, 0, 0,
         0, 0, 0,  0,  0,  0, 0,  0, 0, 1, 0,
         0, 0, 0,  0,  0,  0, 0,  0, 0, 0, 1;

    // 分段连续白噪声 Singer 模型：为底盘赋予物理惯性刚度
    // q_v1_ 是平移加速度方差，q_v2_ 是旋转角加速度方差
    // 独立 z 轴的平移加速度方差为 q_z_
    double a  = (dt * dt * dt * dt) / 4.0;
    double b  = (dt * dt * dt) / 2.0;
    double c  = (dt * dt);
    
    Q << a*q_v1_,    0,    0, b*q_v1_,    0,    0,    0,    0, 0, 0, 0,
            0, a*q_v1_,    0,    0, b*q_v1_,    0,    0,    0, 0, 0, 0,
            0,    0, a*q_z_,    0,    0, b*q_z_,    0,    0, 0, 0, 0,
         b*q_v1_,    0,    0, c*q_v1_,    0,    0,    0,    0, 0, 0, 0,
            0, b*q_v1_,    0,    0, c*q_v1_,    0,    0,    0, 0, 0, 0,
            0,    0, b*q_z_,    0,    0, c*q_z_,    0,    0, 0, 0, 0,
            0,    0,    0,    0,    0,    0, a*q_v2_, b*q_v2_, 0, 0, 0,
            0,    0,    0,    0,    0,    0, b*q_v2_, c*q_v2_, 0, 0, 0,
            0,    0,    0,    0,    0,    0,    0,    0, 0, 0, 0,
            0,    0,    0,    0,    0,    0,    0,    0, 0, 0, 0,
            0,    0,    0,    0,    0,    0,    0,    0, 0, 0, 0;
}


// ============================== 观测模型与数学推导 ==============================


Eigen::Matrix<double, 4, 1> EKF::h(const Eigen::Matrix<double, 11, 1>& X_in)
{
    double x_c = X_in(0), y_c = X_in(1), z_c = X_in(2);
    double yaw = X_in(6);
    
    double dx, dy, dz_offset, theta_offset;
    GetArmorplateParams(X_in, this->armor_id_, dx, dy, dz_offset, theta_offset);

    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    Eigen::Matrix<double, 4, 1> z_pred;
    z_pred(0) = x_c + cos_yaw * dx - sin_yaw * dy;   // x_armor
    z_pred(1) = y_c + sin_yaw * dx + cos_yaw * dy;   // y_armor
    z_pred(2) = z_c + dz_offset;                     // z_armor
    z_pred(3) = yaw + theta_offset;                  // yaw_armor
    NormalizeAngle(z_pred(3)); 

    return z_pred;
}


Eigen::Matrix<double, 4, 11> EKF::ComputeH(const Eigen::Matrix<double, 11, 1>& X_in)
{
    double yaw = X_in(6);
    double dx, dy, dz_offset, theta_offset;
    GetArmorplateParams(X_in, this->armor_id_, dx, dy, dz_offset, theta_offset);

    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    Eigen::Matrix<double, 4, 11> H_mat = Eigen::Matrix<double, 4, 11>::Zero();

    H_mat(0, 0) = 1.0; H_mat(1, 1) = 1.0; H_mat(2, 2) = 1.0; // ∂x_c, ∂y_c, ∂z_c
    
    H_mat(0, 6) = -sin_yaw * dx - cos_yaw * dy; // ∂x_armor / ∂yaw
    H_mat(1, 6) =  cos_yaw * dx - sin_yaw * dy; // ∂y_armor / ∂yaw
    H_mat(3, 6) = 1.0;                          // ∂yaw_armor / ∂yaw

    if (armor_num_ == 4 && (this->armor_id_ == 1 || this->armor_id_ == 3)) 
    {
        H_mat(0, 9)  = -std::cos(yaw + theta_offset); // ∂r2
        H_mat(1, 9)  = -std::sin(yaw + theta_offset);
        H_mat(2, 10) = 1.0;                           // ∂dz
    } 
    else 
    {
        H_mat(0, 8) = -std::cos(yaw + theta_offset); // ∂r1
        H_mat(1, 8) = -std::sin(yaw + theta_offset);
    }

    return H_mat;
}


Eigen::Matrix<double, 4, 1> EKF::h_ypd(const Eigen::Matrix<double, 11, 1>& X_in)
{
    // 获取预测的笛卡尔绝对坐标 XYZ
    Eigen::Matrix<double, 4, 1> z_xyz = this->h(X_in); 
    double pred_x = z_xyz(0), pred_y = z_xyz(1), pred_z = z_xyz(2);
    
    double r2 = pred_x * pred_x + pred_y * pred_y;
    double d  = std::sqrt(r2 + pred_z * pred_z);

    Eigen::Matrix<double, 4, 1> z_ypd;
    z_ypd(0) = std::atan2(pred_y, pred_x);         // los_yaw
    z_ypd(1) = -std::atan2(pred_z, std::sqrt(r2)); // los_pitch
    z_ypd(2) = d;                                  // distance
    z_ypd(3) = z_xyz(3);                           // abs yaw_armor 保持不变

    return z_ypd;
}


Eigen::Matrix<double, 4, 11> EKF::ComputeH_YPD(const Eigen::Matrix<double, 11, 1>& X_in)
{
    // 获取原生 XYZ 雅可比矩阵
    Eigen::Matrix<double, 4, 11> H_xyz = this->ComputeH(X_in);

    // 获取当前预测坐标基点
    Eigen::Matrix<double, 4, 1> z_xyz = this->h(X_in);
    double x = z_xyz(0), y = z_xyz(1), z = z_xyz(2);

    double r2 = x * x + y * y;
    double r  = std::sqrt(r2);
    double d2 = r2 + z * z;
    double d  = std::sqrt(d2);

    // 计算 YPD 模型局部偏导转换微积分矩阵 (Chain Rule)
    Eigen::Matrix3d J_ypd = Eigen::Matrix3d::Zero();
    if (r2 > 1e-6 && d2 > 1e-6) 
    {
        J_ypd(0, 0) = -y / r2;            J_ypd(0, 1) =  x / r2;            J_ypd(0, 2) =  0.0;
        J_ypd(1, 0) = (z * x) / (d2 * r); J_ypd(1, 1) = (z * y) / (d2 * r); J_ypd(1, 2) = -r / d2;
        J_ypd(2, 0) = x / d;              J_ypd(2, 1) = y / d;              J_ypd(2, 2) = z / d;
    }

    Eigen::Matrix<double, 4, 11> H_new = Eigen::Matrix<double, 4, 11>::Zero();
    H_new.block<3, 11>(0, 0) = J_ypd * H_xyz.block<3, 11>(0, 0); 
    H_new.row(3) = H_xyz.row(3); // 第 4 行姿态角导数直通

    return H_new;
}


// ============================== TF 树坐标系广播 ==============================


// 发布 整车中心 -> 四个装甲板中的某个装甲板 的 tf 坐标系变换
void EKF::UpdateCarCenterToArmorplate(std::string child_frame, double x, double y, double z, double roll, double pitch, double yaw)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->current_image_time_;    // 帧头 -> 对齐图像观测时刻
    tf.header.frame_id = "car_center_frame";        // 父坐标系 -> 整车中心坐标系
    tf.child_frame_id = child_frame;                // 子坐标系 -> 四个装甲板坐标系

    // 平移
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    
    // 旋转
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw); // 顺序：roll, pitch, yaw （XYZ） 

    // 设置四元数
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    car_broadcaster_->sendTransform(tf);
}


// 控制逐个发布 整车中心 -> 四个装甲板 分别的 tf 坐标系变换
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


// 发布【父坐标系】->【整车中心坐标系】注意这里忽略了 pitch & roll
// 如果是单机模式，父坐标系是 camera_frame，如果是联调模式，父坐标系是 world_frame
void EKF::UpdateFatherToCarCenter()
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->current_image_time_;    // 帧头 -> 对齐图像观测时刻
    tf.header.frame_id = this->father_frame_;        // 父坐标系 -> 相机坐标系 / 世界坐标系
    tf.child_frame_id = "car_center_frame";         // 子坐标系 -> 整车中心坐标系   

    // 平移
    tf.transform.translation.x = this->X(0);
    tf.transform.translation.y = this->X(1);
    tf.transform.translation.z = this->X(2);
    
    // 旋转 注意这里忽略 pitch & roll
    tf2::Quaternion q;
    q.setRPY(0, 0, this->X(6));  

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    car_broadcaster_->sendTransform(tf);
}


// ============================== 辅助与工具函数 ==============================


// 角度归一化函数，确保角度在 -pi 到 pi 之间，防止跨界跳变
void EKF::NormalizeAngle(double& angle)
{
    angle = std::atan2(std::sin(angle), std::cos(angle));
}


// 根据目前整车状态，计算指定 ID 装甲板的相对中心参数
void EKF::GetArmorplateParams(const Eigen::Matrix<double, 11, 1>& X_in, int id, double& dx, double& dy, double& dz_offset, double& theta_offset)
{
    // 根据 ID 建立装甲板相对整车中心的角度偏差
    // 严格按照 0123 体系：偏差角 = ID * (360 / 装甲板数量)
    theta_offset = id * 2.0 * M_PI / armor_num_;
    NormalizeAngle(theta_offset);

    // 提取实时估计半径，0和2号板取 r1 (X8)，1和3号板取 r2 (X9)。提取高度差
    double truly_radius = X_in(8);
    dz_offset = 0.0;

    if(armor_num_ == 4)
    {
        if (id == 0 || id == 2)
        {
            truly_radius = X_in(8);
            dz_offset = 0.0; // 头尾板直接使用车心高度为基准
        }
        else 
        {
            truly_radius = X_in(9);
            dz_offset = X_in(10); // 侧板加入高低板补偿 dz
        }
    }
    
    // FLU 系中，装甲板中心相对车心的偏差量
    // 装甲板的 +X 轴指向车心，所以它处于车心的反方向
    dx = -truly_radius * std::cos(theta_offset);
    dy = -truly_radius * std::sin(theta_offset);
}


// Debug 打印模块
void EKF::PrintDebugInfo(Eigen::Matrix<double, 4, 1>& innovation)
{
    RCLCPP_INFO(rclcpp::get_logger("ekf_debug"),
        "Innovation: los_yaw=%.4f, los_pitch=%.4f, los_distance=%.4f, euler_yaw=%.4f",
        innovation(0), innovation(1), innovation(2), innovation(3));

    RCLCPP_INFO(rclcpp::get_logger("ekf_debug"), 
        "State: x=%.3f, y=%.3f, z=%.3f, vx=%.3f, vy=%.3f, vz=%.3f, yaw=%.3f, omega=%.3f, r1=%.6f, r2=%.6f, dz=%.6f",
        X(0), X(1), X(2), X(3), X(4), X(5), X(6), X(7), X(8), X(9), X(10));

    RCLCPP_INFO(rclcpp::get_logger("ekf_debug"),
        "P_diag: px=%.4f, py=%.4f, pz=%.4f, pyaw=%.4f",
        P(0,0), P(1,1), P(2,2), P(6,6));

    RCLCPP_INFO(rclcpp::get_logger("ekf_debug"),
        "K_gain: Kx=%.4f, Ky=%.4f, Kz=%.4f, Kyaw=%.4f",
        K(0,2), K(1,0), K(2,1), K(6,3)); 
}
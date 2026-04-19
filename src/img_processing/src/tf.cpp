#include "tf.hpp"
#include "ekf.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


// 有参构造：node 节点指针创建
TF::TF(rclcpp::Node* node): node_(node)
{
    // 创建两个广播器：一个用于 chip_frame 相关（包括静态 camera 变换），一个用于 armorplate_frame
    armorplate_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    // 创建 TF 缓存和监听器（用来查询【世界坐标系 -> 装甲板坐标系】是否可以变换）
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 创建静态变换器并且发布一次【芯片坐标系】->【相机坐标系】的静态变换
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
    publishStaticCameraTransform();


    // ------------------ 进行一个配置文件的读取 -----------------

        std::ifstream file("config.txt");  // 打开配置文件，注意是在工作空间下
        if (!file.is_open()) 
        {
            RCLCPP_ERROR(this->node_->get_logger(), "【 EXIT 】无法打开 config.txt 配置文件。。。。即将退出 tf\n");
            exit(-1);
        }

        std::string each_line;
        int line_count = 0; // 记录行数

        while (std::getline(file, each_line)) 
        {
            // 处理每一行，each_line 即为当前行的字符串
            if (each_line.empty() || each_line[0] == '#' || each_line[0] == '/') continue;
            else
            {
                ++line_count;
                RCLCPP_INFO(this->node_->get_logger(), "已读取配置文件第 %d 个有效行: %s", line_count, each_line.c_str());
                if(line_count == 7)
                {
                    if(each_line == "false" || each_line == "False" || each_line == "FALSE") 
                    {
                        this->SHOW_LOGGER_ERROR = false;
                    }
                    else this->SHOW_LOGGER_ERROR = true;
                    RCLCPP_INFO(this->node_->get_logger(), "【 设置参数 】SHOW_LOGGER_ERROR = %s", each_line.c_str());
                }

                else if(line_count == 8)
                {
                    if(each_line == "false" || each_line == "False" || each_line == "FALSE") 
                    {
                        this->SHOW_RESULT = false;
                    }
                    else this->SHOW_RESULT = true;
                    RCLCPP_INFO(this->node_->get_logger(), "【 设置参数 】SHOW_RESULT = %s", each_line.c_str());
                }

            }
        }

        if(line_count < 8)
        {
            RCLCPP_ERROR(this->node_->get_logger(), "配置文件的有效行数不足7行, 检查配置文件。即将退出 core 节点\n");
            exit(-1);
        }
        else
        {
            RCLCPP_INFO(this->node_->get_logger(), "【 设置参数完成 】TF ALL SET! 共设置了 %d 个有效参数", line_count);
        }

        file.close();
}




// 02【芯片坐标系】->【相机坐标系】当前相机位姿的坐标系 -> 静态，和芯片坐标系可以视为刚体，用 t 向量
void TF::publishStaticCameraTransform()
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = rclcpp::Time(0); // 帧头 -> 设置成 0
    tf.header.frame_id = "chip_frame"; // 父坐标系 -> 芯片坐标系
    tf.child_frame_id = "camera_frame"; // 子坐标系 -> 相机坐标系

    // 静态平移：芯片坐标系下相机的位置（前60mm，下30mm）
    tf.transform.translation.x = 0.06;   // 米
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = -0.03;

    // 无旋转，所以用单位四元数
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    // 用静态广播器 广播
    static_broadcaster_->sendTransform(tf);

    RCLCPP_INFO_ONCE(this->node_->get_logger(), "已发布静态变换【芯片坐标系 -> 相机坐标系】，平移: x=%.3f m, y=%.3f m, z=%.3f m", tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
}



// 03【相机坐标系 -> 装甲板坐标系】当前装甲板位姿的坐标系 -> 动态
void TF::updateCameraToArmorplate(Eigen::Matrix3d R, Eigen::Vector3d t)
{
    // 已经在 serial_driver 里判断过 PNP 是否和上次收到的完全相同了

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->node_->now();
    tf.header.frame_id = "camera_frame"; // 父坐标系 -> 相机坐标系
    tf.child_frame_id = "armorplate_frame"; // 子坐标系 -> 装甲板坐标系


    // 1. mm -> m（因为 tf 单位都是 m）
    tf.transform.translation.x = t[0] / 1000.0; // x
    tf.transform.translation.y = t[1] / 1000.0; // y
    tf.transform.translation.z = t[2] / 1000.0; // z

        
    // 2. 旋转矩阵 -> 四元数
    Eigen::Quaterniond q(R); 
    q.normalize();  // 归一化

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    armorplate_broadcaster_->sendTransform(tf);

}



// ---------- 辅助函数 ----------
// 根据世界坐标系下装甲板的预测坐标，推算相机对准时云台应转动的角度

void TF::getFixCameraAngle(float X, float Y, float Z, float& pitch, float& yaw)
{
    // 相机相对于芯片的固定平移（米）
    const double dx = 0.06;
    const double dz = -0.03;
    Eigen::Vector3d t(dx, 0.0, dz);   // 相机在芯片坐标系下的位置
    Eigen::Vector3d e(1.0, 0.0, 0.0); // 芯片的 X 轴（光轴方向）
    Eigen::Vector3d P(X, Y, Z);       // 装甲板在世界坐标系下的位置

    // 第一步：粗略方向（忽略相机偏移）
    Eigen::Vector3d d0 = P.normalized();

    // 第二步：计算从 e 到 d0 的旋转轴和角度
    Eigen::Vector3d axis = e.cross(d0);
    double angle = std::acos(e.dot(d0));
    
    Eigen::Vector3d C0; // 估算的相机位置
    if (axis.norm() < 1e-8) {
        // e 与 d0 平行（目标正好在正前方或正后方），此时旋转轴任意，旋转角0
        C0 = t; // 芯片未旋转，相机位置就是 t 本身
    } else {
        Eigen::AngleAxisd rot(angle, axis.normalized()); // 旋转矩阵
        C0 = rot * t; // 将相机偏移旋转到世界系
    }

    // 第三步：修正方向
    Eigen::Vector3d d1 = (P - C0).normalized(); // 方向向量

    // 第四步：转换为偏航和俯仰
    pitch = -std::atan2(d1.z(), std::sqrt(d1.x() * d1.x() + d1.y() * d1.y())) * 180.0 / M_PI;   
    yaw = std::atan2(d1.y(), d1.x()) * 180.0 / M_PI;
}



// 查询【世界坐标系】->【装甲板坐标系】是否可以变换
// 通过引用回传滤波后的最终结果，返回1或者0表示是否有效
bool TF::getWorldToArmorplateTransform(Eigen::Vector3d& armorplate_center, double& yaw_armor)
{
    geometry_msgs::msg::TransformStamped transform_world_armorplate; // 世界 -> 装甲板
    try 
    {
        transform_world_armorplate = tf_buffer_->lookupTransform("world_frame", "armorplate_frame", tf2::TimePointZero);
    } 
    catch (tf2::TransformException &ex) 
    {
        RCLCPP_ERROR_EXPRESSION(node_->get_logger(), this->SHOW_LOGGER_ERROR, "【世界坐标系 -> 装甲板 坐标系】TF lookup failed: %s", ex.what());
        return false; 
    }


    ///////////////////////////////////// 世界 -> 装甲板 的 位置 ////////////////////////////////////////


    // 获得 世界 -> 装甲板 的平移向量 
    armorplate_center[0] = transform_world_armorplate.transform.translation.x;
    armorplate_center[1] = transform_world_armorplate.transform.translation.y;
    armorplate_center[2] = transform_world_armorplate.transform.translation.z;

    RCLCPP_INFO_EXPRESSION(node_->get_logger(), this->SHOW_RESULT, "查询到【世界坐标系 -> 装甲板坐标系】的平移向量: X=%.7f m, Y=%.7f m, Z=%.7f m", armorplate_center[0], armorplate_center[1], armorplate_center[2]);

    // 旋转矩阵
    tf2::Quaternion q(
            transform_world_armorplate.transform.rotation.x,
            transform_world_armorplate.transform.rotation.y,
            transform_world_armorplate.transform.rotation.z,
            transform_world_armorplate.transform.rotation.w);
    double roll_armor, pitch_armor;
    tf2::Matrix3x3(q).getRPY(roll_armor, pitch_armor, yaw_armor);


/*
    // 调用滤波算法，滤波 世界 -> 装甲板 的位置

    
    
    this->kf_position_->getKalman(X_armor, Y_armor, Z_armor, dt); // 更新 KF 的状态


    // 写在外面不更新没问题，因为是直接赋值
    double X_filt, Y_filt, Z_filt;
    double X_prev, Y_prev, Z_prev;

    // 前几帧数据用来让 KF 稳定下来，不急着滤波，直接用测量值计算位置
    if(count > 2) 
    {
        this->kf_position_->getData(X_filt, Y_filt, Z_filt); // 获取滤波后的结果
        RCLCPP_INFO_EXPRESSION(node_->get_logger(), this->SHOW_RESULT, "滤波后【世界坐标系 -> 装甲板坐标系】的平移向量: X=%.7f m, Y=%.7f m, Z=%.7f m", X_filt, Y_filt, Z_filt);

        this->kf_position_->getPredict(X_prev, Y_prev, Z_prev, 0.010); // 预测未来 10ms 的位置，看看趋势
        RCLCPP_INFO_EXPRESSION(node_->get_logger(), this->SHOW_RESULT, "预测未来 10ms 后【世界坐标系 -> 装甲板坐标系】的平移向量: X=%.7f m, Y=%.7f m, Z=%.7f m", X_prev, Y_prev, Z_prev);

        // 计算预测位置和当前测量位置的距离，看看 KF 的预测是否靠谱。如果距离超过 0.50m 的阈值，说明预测不准，直接用滤波值计算角度
        double dx = X_prev - X_armor;
        double dy = Y_prev - Y_armor;
        double dz = Z_prev - Z_armor;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if(distance > 0.30) // 如果预测位置和当前测量位置相差太大，说明预测不准，直接用滤波值计算角度
        {
            RCLCPP_WARN(this->node_->get_logger(), "预测位置和当前测量位置相差 %.3f m, 超过 0.30 m 的阈值，说明预测不准，直接用测量值计算角度", distance);
            X_prev = X_filt;
            Y_prev = Y_filt;
            Z_prev = Z_filt;
        }
    }
    else // 不够 2 帧就先用原来的值
    {
        X_prev = X_armor;
        Y_prev = Y_armor;
        Z_prev = Z_armor;
    }


    // EKF 预测
    double X_ekf = 0.0;
    double Y_ekf = 0.0;
    double Z_ekf = 0.0;
    double X_ekf_center = 0.0;
    double Y_ekf_center = 0.0;
    double Z_ekf_center = 0.0;


    this->ekf_->getKalman(X_filt, Y_filt, Z_filt, yaw_armor, 3, dt);
    this->ekf_->getArmorPredict(X_ekf, Y_ekf, Z_ekf, 3, 0.20);
    this->ekf_->getCenterPredict(X_ekf_center, Y_ekf_center, Z_ekf_center, 0.20);
    
    double pitch_diff = -std::atan2(Z_ekf, std::sqrt(X_ekf * X_ekf + Y_ekf * Y_ekf)) * 180.0 / M_PI;   
    double yaw_diff = std::atan2(Y_ekf, X_ekf) * 180.0 / M_PI;
    pitch = pitch_diff;
    yaw = yaw_diff;


    ///////////////////////////////////// 得到最终角度 ////////////////////////////////////////
    

    // 用 t 计算 滤波后的装甲板相对于世界的【偏差角】pitch & yaw

    // atan2(y, x) 的符号只由 y 决定，与 x 无关。

    // pitch + 表示目标在相机下方。当△z为-时，结果为-。装甲板在下方，需要pitch为+，所以要取负号
    // yaw + 表示目标在相机左侧。因为当△y为-时，结果为-。装甲板在右方，需要yaw为-，没问题

    // double pitch_diff = -std::atan2(Z_prev, std::sqrt(X_prev * X_prev + Y_prev * Y_prev)) * 180.0 / M_PI;   
    // double yaw_diff = std::atan2(Y_prev, X_prev) * 180.0 / M_PI;
    // pitch = pitch_diff;
    // yaw = yaw_diff;
    
    // float pitch_genbil = 0, yaw_genbil = 0;
    // getFixCameraAngle(X_prev, Y_prev, Z_prev, pitch_genbil, yaw_genbil);

    // pitch = pitch_genbil;
    // yaw = yaw_genbil;


    // // 对最终结果再进行一次 KF 滤波，看看能不能更稳定一些
    // this->kf_data_->getKalman(pitch, yaw); // 更新 KF 的状态

    // // 前几帧数据用来让 KF 稳定下来，不急着滤波，直接用计算值就好
    // if(count > 5)
    // {
    //     double pitch_filt, yaw_filt;
    //     this->kf_data_->getData(pitch_filt, yaw_filt); // 获取滤波后的结果
    //     pitch = pitch_filt;
    //     yaw = yaw_filt;
    //     RCLCPP_INFO_EXPRESSION(node_->get_logger(), this->SHOW_RESULT, "滤波后【世界坐标系 -> 装甲板坐标系】的平移向量: X=%.7f m, Y=%.7f m, Z=%.7f m", X_filt, Y_filt, Z_filt);
    // }
*/

    return true;

}

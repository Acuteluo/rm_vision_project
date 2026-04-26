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


    // 从参数服务器读取初始参数
    this->SHOW_LOGGER_ERROR = node_->get_parameter("tf.show_logger_error").as_bool();
    this->SHOW_RESULT = node_->get_parameter("tf.show_result").as_bool();
    this->father_frame = node_->get_parameter("is_standalone").as_bool() ? "camera_frame" : "world_frame"; // 根据单机/联调模式设置父坐标系名字

    RCLCPP_INFO(node_->get_logger(), "TF 参数已加载: SHOW_LOGGER_ERROR=%d, SHOW_RESULT=%d, father_frame=%s", 
                SHOW_LOGGER_ERROR, SHOW_RESULT, father_frame.c_str());


}


// 在参数变化时立即刷新
void TF::updateParamsFromServer()
{
    if (!node_) return;
    this->SHOW_LOGGER_ERROR = node_->get_parameter("tf.show_logger_error").as_bool();
    this->SHOW_RESULT = node_->get_parameter("tf.show_result").as_bool();
    this->father_frame = node_->get_parameter("is_standalone").as_bool() ? "camera_frame" : "world_frame"; // 根据单机/联调模式设置父坐标系名字
}




// 02【芯片坐标系】->【相机坐标系】当前相机位姿的坐标系 -> 静态，和芯片坐标系可以视为刚体，用 t 向量
// 但是，由于姿态是姿态，可以平移，和位置无关，所以直接把相机当做在轴上转，直接拿 imu 的数据 作为姿态喵
// 所以目前，这个直接没有任何作用
void TF::publishStaticCameraTransform()
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = rclcpp::Time(0); // 帧头 -> 设置成 0, 表示无论什么时候查，这个矩阵都永远有效
    tf.header.frame_id = "chip_frame"; // 父坐标系 -> 芯片坐标系
    tf.child_frame_id = "camera_frame"; // 子坐标系 -> 相机坐标系

    // 静态平移：芯片坐标系下相机的位置（前60mm，下30mm -> 0mm）
    tf.transform.translation.x = 0.0;   // m
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0; // 注意！位置和姿态是分开的，既然我只能拿到芯片的姿态欧拉角，那芯片在轴上哪个位置，难道还重要吗！

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
// 发布归发布，为的只是可视化！由于刚发完很快就要查根本来不及，我们直接在另一个tf函数里，用pnp得到的R|t直接查father->装甲板，保证能查到！
void TF::updateCameraToArmorplate(Eigen::Matrix3d R, Eigen::Vector3d t, rclcpp::Time time_stamp)
{
    // 已经在 serial_driver 里判断过 PNP 是否和上次收到的完全相同了

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = time_stamp; // 帧头 -> 直接使用传入的时间戳，保证和图像时间戳对齐
    tf.header.frame_id = "camera_frame"; // 父坐标系 -> 相机坐标系
    tf.child_frame_id = "armorplate_frame"; // 子坐标系 -> 装甲板坐标系


    // 1. tf 单位都是 m，pnp 结算里已经换成了 m 了！
    tf.transform.translation.x = t[0]; // x
    tf.transform.translation.y = t[1]; // y
    tf.transform.translation.z = t[2]; // z

        
    // 2. 旋转矩阵 -> 四元数
    Eigen::Quaterniond q(R); 
    q.normalize();  // 归一化

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    armorplate_broadcaster_->sendTransform(tf);

}



// 查询【相机坐标系】-> 【父坐标系】是否可以变换
// 单机模式时父坐标系是 camera_frame，联调模式时父坐标系是 world_frame
// 最后一个参数是查询时间戳，保证和图像时间戳对齐
bool TF::getCameraToWorldTransform(tf2::Transform& T_world_to_cam_point, rclcpp::Time time_stamp)
{
    geometry_msgs::msg::TransformStamped transform;
    try 
    {
        // 查询 world_frame 到 camera_frame 的变换
        // transform = tf_buffer_->lookupTransform("camera_frame", this->father_frame, tf2::TimePointZero);
        transform = tf_buffer_->lookupTransform("camera_frame", this->father_frame, time_stamp);
    }
    catch (tf2::TransformException &ex) 
    {
        RCLCPP_ERROR_EXPRESSION(node_->get_logger(), this->SHOW_LOGGER_ERROR, "【%s 坐标系 -> 相机 坐标系】 TF lookup failed: %s", this->father_frame, ex.what());
        return false;
    }

    // 将 geometry_msgs::Transform 转换为 tf2::Transform
    tf2::fromMsg(transform.transform, T_world_to_cam_point);
    return true;
}


// 查询【世界坐标系】->【芯片坐标系】，通过引用回传角度值，返回1或者0表示是否有效
bool TF::getWorldToChipTransform(double& pitch_chip, double& yaw_chip, rclcpp::Time time_stamp)
{
    geometry_msgs::msg::TransformStamped tf_world_chip;
    try 
    {
        // 查询 world_frame -> chip_frame 的变换
        tf_world_chip = tf_buffer_->lookupTransform("world_frame", "chip_frame", time_stamp);
    } 
    catch (tf2::TransformException &ex) 
    {
        // 查不到时使用节流打印，防止刷屏
        RCLCPP_WARN(this->node_->get_logger(), "获取【世界坐标系】->【芯片坐标系】失败: %s", ex.what());
        return false; 
    }

    // 提取四元数
    tf2::Quaternion q_chip(
        tf_world_chip.transform.rotation.x,
        tf_world_chip.transform.rotation.y,
        tf_world_chip.transform.rotation.z,
        tf_world_chip.transform.rotation.w
    );

    // 转换为欧拉角 (Roll, Pitch, Yaw)
    double roll_chip;
    tf2::Matrix3x3(q_chip).getRPY(roll_chip, pitch_chip, yaw_chip);

    // 弧度 (rad) 转为 角度 (degree)
    pitch_chip = pitch_chip * 180.0 / M_PI;
    yaw_chip = yaw_chip * 180.0 / M_PI;

    return true;
}


// // 查询【父坐标系】->【装甲板坐标系】是否可以变换
// // 单机模式时父坐标系是 camera_frame，联调模式时父坐标系是 world_frame
// // 通过引用回传滤波后的最终结果，返回1或者0表示是否有效
// // 最后一个参数是查询时间戳，保证和图像时间戳对齐
// bool TF::getFatherToArmorplateTransform(Eigen::Vector3d& armorplate_center, double& yaw_armor, rclcpp::Time time_stamp)
// {
//     geometry_msgs::msg::TransformStamped transform_world_armorplate; // 世界 -> 装甲板
//     try 
//     {
//         // transform_world_armorplate = tf_buffer_->lookupTransform(this->father_frame, "armorplate_frame", tf2::TimePointZero);
//         transform_world_armorplate = tf_buffer_->lookupTransform(this->father_frame, "armorplate_frame", tf2::TimePointZero);
//     } 
//     catch (tf2::TransformException &ex) 
//     {
//         RCLCPP_ERROR_EXPRESSION(node_->get_logger(), this->SHOW_LOGGER_ERROR, "【%s -> 装甲板 坐标系】TF lookup failed: %s", this->father_frame, ex.what());
//         return false; 
//     }


//     ///////////////////////////////////// 世界 -> 装甲板 的 位置 ////////////////////////////////////////


//     // 获得 世界 -> 装甲板 的平移向量 
//     armorplate_center[0] = transform_world_armorplate.transform.translation.x;
//     armorplate_center[1] = transform_world_armorplate.transform.translation.y;
//     armorplate_center[2] = transform_world_armorplate.transform.translation.z;

//     RCLCPP_INFO_EXPRESSION(node_->get_logger(), this->SHOW_RESULT, "查询到【世界坐标系 -> 装甲板坐标系】的平移向量: X=%.7f m, Y=%.7f m, Z=%.7f m", armorplate_center[0], armorplate_center[1], armorplate_center[2]);

//     // 旋转矩阵
//     tf2::Quaternion q(
//             transform_world_armorplate.transform.rotation.x,
//             transform_world_armorplate.transform.rotation.y,
//             transform_world_armorplate.transform.rotation.z,
//             transform_world_armorplate.transform.rotation.w);

//     // double roll_armor, pitch_armor;
//     // tf2::Matrix3x3(q).getRPY(roll_armor, pitch_armor, yaw_armor);

//     // 1. 转成矩阵
//     tf2::Matrix3x3 mat(q);

//     // 2. 提取第一列的前两个元素 (世界坐标系下的 X 和 Y 投影) 注意这里是 R 矩阵！
//     double vec_x = mat[0][0]; 
//     double vec_y = mat[1][0]; 

//     // 3. 直接用 atan2 算出偏航角
//     yaw_armor = std::atan2(vec_y, vec_x);

//     return true;
// }



bool TF::getFatherToArmorplateTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, Eigen::Vector3d& armorplate_center, double& yaw_armor, rclcpp::Time time_stamp)
{
    // 1. 如果是单机模式，父坐标系就是 camera_frame，直接使用 PnP 的计算结果，0延迟！
    if (this->father_frame == "camera_frame")
    {
        armorplate_center = t;
        yaw_armor = std::atan2(R(1, 0), R(0, 0)); // 直接求旋转矩阵第一列的偏航角
        return true;
    }

    else
    {
        // 2. 如果是联调模式，父坐标系是 world_frame。只查询【相机】在世界中的姿态！
        tf2::Transform T_world_to_cam; 

        // 调用我们已经写好的函数查 TF，如果查不到直接返回 false，报错日志在那个函数里已经打印了
        if (!this->getCameraToWorldTransform(T_world_to_cam, time_stamp)) 
        {
            return false; 
        }

        // 【数学魔法】：getCameraToWorld 拿到的是 World -> Camera 的映射
        // 我们将其求逆，就得到了 Camera -> World 的映射 (即相机在世界中的绝对姿态)
        tf2::Transform T_cam_to_world = T_world_to_cam.inverse();

        // 提取相机在世界系下的平移和旋转
        Eigen::Vector3d t_father_cam(
            T_cam_to_world.getOrigin().x(), 
            T_cam_to_world.getOrigin().y(), 
            T_cam_to_world.getOrigin().z()
        );
        
        // 提取相机在世界系下的旋转，并转为 Eigen 矩阵
        tf2::Matrix3x3 mat(T_cam_to_world.getRotation());
        Eigen::Matrix3d R_father_cam;
        for (int i = 0; i < 3; i++) 
        {
            for (int j = 0; j < 3; j++) 
            {
                R_father_cam(i, j) = mat[i][j];
            }
        }

        // =================================================================
        // 核心数学映射：世界坐标 = 相机在世界的原点 + (相机在世界的朝向 * 相对坐标)
        // =================================================================
        armorplate_center = t_father_cam + R_father_cam * t;

        // 姿态叠加，并提取绝对 Yaw 角
        Eigen::Matrix3d R_father_armor = R_father_cam * R;
        yaw_armor = std::atan2(R_father_armor(1, 0), R_father_armor(0, 0));

        RCLCPP_INFO_EXPRESSION(node_->get_logger(), this->SHOW_RESULT, "装甲板在【世界坐标系】的坐标: X=%.3f, Y=%.3f, Z=%.3f", armorplate_center[0], armorplate_center[1], armorplate_center[2]);

        return true;
    }
    
}
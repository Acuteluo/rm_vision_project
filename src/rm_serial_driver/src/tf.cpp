#include "rm_serial_driver/tf.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rm_serial_driver
{

// 有参构造：node 节点指针创建
TF::TF(rclcpp::Node* node): node_(node)
{
    // 创建两个广播器：一个用于 chip_frame 相关（包括静态 camera 变换），一个用于 armorplate_frame
    chip_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
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



// 01【世界坐标系】->【芯片坐标系】当前芯片位姿的坐标系 -> 动态，用 R 矩阵
void TF::updateWorldToChip(double roll, double pitch, double yaw)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->node_->now(); // 帧头 -> 直接获取当前时刻
    tf.header.frame_id = "world_frame"; // 父坐标系 -> 世界坐标系
    tf.child_frame_id = "chip_frame"; // 子坐标系 -> 芯片坐标系

    // 先忽略 t
    tf.transform.translation.x = 0.00;
    tf.transform.translation.y = 0.00;
    tf.transform.translation.z = 0.00;
    

    // 欧拉角（度）->（弧度）再转四元数

    // 先把 度 -> 弧度，注意 setRPY() 需要弧度
    double roll_rad = roll * M_PI / 180.0;
    double pitch_rad = pitch * M_PI / 180.0;
    double yaw_rad = yaw * M_PI / 180.0;

    // 然后转四元数
    tf2::Quaternion q;
    q.setRPY(roll_rad, pitch_rad, yaw_rad);  // 顺序：roll, pitch, yaw （XYZ） 

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    chip_broadcaster_->sendTransform(tf);
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
// 直接传入 serial_driver 节点拿到的 pnp 的 msg 消息的引用
void TF::updateCameraToArmorplate(const serial_driver_interfaces::msg::SendPNPInfo& msg)
{
    // 已经在 serial_driver 里判断过 PNP 是否和上次收到的完全相同了

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = msg.header.stamp; // 帧头
    tf.header.frame_id = "camera_frame"; // 父坐标系 -> 相机坐标系
    tf.child_frame_id = "armorplate_frame"; // 子坐标系 -> 装甲板坐标系


    // 1. mm -> m（因为 tf 单位都是 m）
    tf.transform.translation.x = msg.tvec[0] / 1000.0; // x
    tf.transform.translation.y = msg.tvec[1] / 1000.0; // y
    tf.transform.translation.z = msg.tvec[2] / 1000.0; // z

        
    // 2. 转存到 Eigen 矩阵，方便转换成四元数
    Eigen::Matrix3d R_eigen;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            R_eigen(i, j) = msg.matrix_r[i * 3 + j];
        }
            
    }
        

    // 3. 旋转矩阵 -> 四元数
    Eigen::Quaterniond q(R_eigen); 
    q.normalize();  // 归一化

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    armorplate_broadcaster_->sendTransform(tf);

}


// 查询【世界坐标系 -> 装甲板坐标系】是否可以变换，可以就变换并滤波，通过引用回传结果，返回1或者0表示是否有效
bool TF::getTransform(float& pitch, float& yaw)
{

    geometry_msgs::msg::TransformStamped transform;
    try 
    {
        transform = tf_buffer_->lookupTransform("world_frame", "armorplate_frame", tf2::TimePointZero);
        // transform = tf_buffer_->lookupTransform("world_frame", "armorplate_frame", this->node_->now());
        // transform = tf_buffer_->lookupTransform("world_frame", "armorplate_frame", this->node_->now(), tf2::durationFromSec(0.05));
    } 
    catch (tf2::TransformException &ex) 
    {
        RCLCPP_ERROR_EXPRESSION(node_->get_logger(), this->SHOW_LOGGER_ERROR, "【世界坐标系 -> 装甲板坐标系】TF lookup failed: %s", ex.what());
        return false; 
    }

    // 获得平移向量
    double X, Y, Z; 
    X = transform.transform.translation.x;
    Y = transform.transform.translation.y;
    Z = transform.transform.translation.z;

    RCLCPP_INFO_EXPRESSION(node_->get_logger(), this->SHOW_RESULT, "查询到【世界坐标系 -> 装甲板坐标系】的平移向量: X=%.7f m, Y=%.7f m, Z=%.7f m", X, Y, Z);


    // to do
    // 这里调用滤波算法


    // 用 t 计算【绝对角】pitch & yaw

    // atan2(y, x) 的符号只由 y 决定，与 x 无关。

    // pitch + 表示目标在相机下方。当△z为-时，结果为-。装甲板在下方，需要pitch为+，所以要取负号
    // yaw + 表示目标在相机左侧。因为当△y为-时，结果为-。装甲板在右方，需要yaw为-，没问题
    
    pitch = -std::atan2(Z, std::sqrt(X * X + Y * Y)) * 180.0 / M_PI;   
    yaw = std::atan2(Y, X) * 180.0 / M_PI;

    return true;

}


}// namespace rm_serial_driver
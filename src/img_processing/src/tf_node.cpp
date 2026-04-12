#include "prepare_algorithm.h"
#include "serial_driver_interfaces/msg/serial_driver.hpp" // 串口消息 发送方 的 话题类型 为 [serial_driver] 类
#include "serial_driver_interfaces/msg/receive_data.hpp" // 串口消息 接收方 的 话题类型 为 [receive_data] 类
#include "serial_driver_interfaces/msg/send_pnp_info.hpp" // pnp消息 接收方 的 话题类型 为 [send_pnp_info] 类

#include <tf2/time.h>

class TFNode : public rclcpp::Node
{
public:
    TFNode(): Node("tf_node"), last_publish_time_(0, 0, this->get_clock()->get_clock_type())
    {
        RCLCPP_INFO(this->get_logger(), "TFNode 节点创建成功! ");

        // 订阅当前芯片位姿
        chip_sub_ = this->create_subscription<serial_driver_interfaces::msg::ReceiveData>("/receive_data", 10, std::bind(&TFNode::ChipCallback, this, std::placeholders::_1));


        // 订阅 PnP 
        pnp_sub_ = this->create_subscription<serial_driver_interfaces::msg::SendPNPInfo>("/send_pnp_info", 10, std::bind(&TFNode::PNPCallback, this, std::placeholders::_1));


        // 初始化动态TF广播器（必须！否则空指针）
        chip_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        armorplate_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // 发布静态变换
        // publishStaticCameraTransform();

        // 启动定时器，每500ms发布一次 chip_frame -> camera_frame
    camera_static_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&TFNode::publishCameraTransformPeriodically, this)
    );
    RCLCPP_INFO(this->get_logger(), "已启动周期性发布 chip_frame -> camera_frame (周期 500ms)");


        // 创建缓存对象
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        // 创建监听器，绑定缓存对象
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        // 发布串口消息
        pub_ = this->create_publisher<serial_driver_interfaces::msg::SerialDriver>("/serial_driver", 10);
    
        //【修改】设置发布频率限制，防止过度发布（最大100Hz）
        publish_rate_ms_ = 10; // 10ms = 100Hz

        RCLCPP_INFO(this->get_logger(), "TFNode 已经初始化完成，正在等待接收芯片位姿和 PnP 消息...");
    }


    // 得到【世界坐标系 -> 装甲板坐标系】的转换，并发送串口消息
    void sendSerialDriverMessage()
    {
        // 【修改】1. 发送消息频率限制：如果距离上次发布时间小于设定间隔，则不发布
        auto current_time = this->now();
        auto time_diff = (current_time - this->last_publish_time_).nanoseconds();
        if (time_diff < publish_rate_ms_ * 1000000) // 转换为纳秒来对比
        { 
            RCLCPP_WARN(this->get_logger(), "【 消息SKIP 】time_diff = %.2f ms < %.2f ms", time_diff / 1000000.0, publish_rate_ms_);
            return;
        }


        geometry_msgs::msg::TransformStamped transform;
        try 
        {
            transform = buffer_->lookupTransform("world_frame", "armorplate_frame", tf2::TimePointZero);
        } 
        catch (tf2::TransformException &ex) 
        {
            RCLCPP_ERROR(this->get_logger(), "【 返回 】TF lookup failed: %s", ex.what());
            return;
        }

        // 更新上一次发布的时间
        this->last_publish_time_ = current_time;


        // 获得平移向量
        double X, Y, Z; 
        X = transform.transform.translation.x;
        Y = transform.transform.translation.y;
        Z = transform.transform.translation.z;


        // to do
        // 这里调用滤波算法


        // 用 t 计算【绝对角】pitch & yaw

        // atan2(y, x) 的符号只由 y 决定，与 x 无关。
        // yaw + 表示目标在相机左侧。因为当△y为-时，结果为-。装甲板在右方，需要yaw为-，没问题
        // pitch + 表示目标在相机下方。当△z为-时，结果为-。装甲板在下方，需要pitch为+，所以要取负号
        double t_absolute_yaw = std::atan2(Y, X) * 180.0 / CV_PI;
        double t_absolute_pitch = -std::atan2(Z, std::sqrt(X * X + Y * Y)) * 180.0 / CV_PI;   



        //////////////////////// 发送串口消息 ////////////////////////

        auto serial_driver = serial_driver_interfaces::msg::SerialDriver();

        // 此时肯定有装甲板，有有效信息，直接发送
        serial_driver.pitch = t_absolute_pitch; // 发送 置信度max 装甲板的 绝对角 pitch
        serial_driver.yaw = t_absolute_yaw; // 发送 置信度max 的装甲板的 绝对角 yaw
        
        pub_->publish(serial_driver); // 发布消息 到 serial_driver 话题
        
        RCLCPP_INFO(this->get_logger(), "【 发布 】tf_node 发布了消息（绝对角）: pitch=%.2f yaw=%.2f", serial_driver.pitch, serial_driver.yaw);

    }

private:

    // 【芯片坐标系 -> 相机坐标系】当前相机位姿的坐标系 -> 静态，和芯片坐标系可以视为刚体，用 t 向量
    // void publishStaticCameraTransform()
    // {
    //     static tf2_ros::StaticTransformBroadcaster static_broadcaster(this);
    //     geometry_msgs::msg::TransformStamped static_transform;

    //     // 静态变换
    //     static_transform.header.stamp = rclcpp::Time(0); // 只发布一次，不在意帧头
    //     static_transform.header.frame_id = "chip_frame"; // 父坐标系 -> 芯片坐标系
    //     static_transform.child_frame_id = "camera_frame"; // 子坐标系 -> 相机坐标系

    //     // 平移：芯片坐标系下，相机中心位于芯片坐标系的 前55mm、下30mm
    //     static_transform.transform.translation.x = 0.055;  // 米
    //     static_transform.transform.translation.y = 0.00;
    //     static_transform.transform.translation.z = -0.03;    // 米

    //     // 无旋转，所以用单位四元数
    //     static_transform.transform.rotation.x = 0.0;
    //     static_transform.transform.rotation.y = 0.0;
    //     static_transform.transform.rotation.z = 0.0;
    //     static_transform.transform.rotation.w = 1.0;

    //     static_broadcaster.sendTransform(static_transform);

    //     RCLCPP_INFO(this->get_logger(), "【 发布 】静态坐标系变换已发布: Static transform chip_frame -> camera_frame published");
    // }

    void publishCameraTransformPeriodically()
{
    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = this->now();
    static_transform.header.frame_id = "chip_frame";
    static_transform.child_frame_id = "camera_frame";

    // 平移：芯片坐标系下，相机中心位于芯片坐标系的 前55mm、下30mm
    static_transform.transform.translation.x = 0.055;   // 米
    static_transform.transform.translation.y = 0.00;
    static_transform.transform.translation.z = -0.03;   // 米

    // 无旋转，单位四元数
    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;

    // 使用已有的普通广播器 chip_tf_broadcaster_（注意这个广播器在构造函数中已初始化）
    chip_tf_broadcaster_->sendTransform(static_transform);

    // 可选：降低打印频率，避免刷屏（比如每发布10次打印一次）
    static int count = 0;
    if (++count % 10 == 0) {
        RCLCPP_DEBUG(this->get_logger(), "周期性发布 chip_frame -> camera_frame");
    }
}


    // 【世界坐标系 -> 芯片坐标系】当前芯片位姿的坐标系 -> 动态，用 R 矩阵
    void ChipCallback(const serial_driver_interfaces::msg::ReceiveData msg)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = msg.header.stamp; // 帧头
        tf.header.frame_id = "world_frame"; // 父坐标系 -> 世界坐标系
        tf.child_frame_id = "chip_frame"; // 子坐标系 -> 芯片坐标系

        // 先忽略 t
        tf.transform.translation.x = 0.00;
        tf.transform.translation.y = 0.00;
        tf.transform.translation.z = 0.00;
        

        // 欧拉角（度）->（弧度）再转四元数

        // 先度转弧度
        double roll_rad = msg.roll * M_PI / 180.0;
        double pitch_rad = msg.pitch * M_PI / 180.0;
        double yaw_rad = msg.yaw * M_PI / 180.0;

        // 然后转四元数
        tf2::Quaternion q;
        q.setRPY(roll_rad, pitch_rad, yaw_rad);  // 顺序：roll, pitch, yaw （XYZ） 

        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        chip_tf_broadcaster_->sendTransform(tf);
    }


    // 【相机坐标系 -> 装甲板坐标系】当前装甲板位姿的坐标系 -> 动态
    void PNPCallback(const serial_driver_interfaces::msg::SendPNPInfo msg)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = msg.header.stamp; // 帧头
        tf.header.frame_id = "camera_frame"; // 父坐标系 -> 相机坐标系
        tf.child_frame_id = "armorplate_frame"; // 子坐标系 -> 装甲板坐标系

        // 1. mm -> m（因为 tf 单位都是 m）
        tf.transform.translation.x = msg.tvec[0] / 1000.0; // x
        tf.transform.translation.y = msg.tvec[1] / 1000.0; // y
        tf.transform.translation.z = msg.tvec[2] / 1000.0; // z

         

        // 4. 转存到 Eigen 矩阵，方便转换成四元数
        Eigen::Matrix3d R_eigen;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R_eigen(i, j) = msg.matrix_r[i * 3 + j];
            }
                
        }
            
        // 5. 旋转矩阵 -> 四元数
        Eigen::Quaterniond q(R_eigen); 
        q.normalize();  // 归一化

        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        armorplate_tf_broadcaster_->sendTransform(tf);

        // 每次 PnP 更新后，立即发送世界坐标系下的装甲板位姿
        sendSerialDriverMessage();
    }


    rclcpp::Subscription<serial_driver_interfaces::msg::ReceiveData>::SharedPtr chip_sub_; // 订阅 当前芯片位姿 话题
    rclcpp::Subscription<serial_driver_interfaces::msg::SendPNPInfo>::SharedPtr pnp_sub_; // 订阅 PnP 话题
    std::unique_ptr<tf2_ros::TransformBroadcaster> chip_tf_broadcaster_; // 发布 芯片坐标系
    std::unique_ptr<tf2_ros::TransformBroadcaster> armorplate_tf_broadcaster_; // 发布 装甲板坐标系

    rclcpp::Publisher<serial_driver_interfaces::msg::SerialDriver>::SharedPtr pub_; // 发布串口所需信息
 
    std::shared_ptr<tf2_ros::Buffer> buffer_; // 缓存对象
    std::shared_ptr<tf2_ros::TransformListener> listener_; // 监听器


    rclcpp::Time last_publish_time_; // 上一次发布的时间
    double publish_rate_ms_; // 发布频率限制（毫秒）


     // 新增：用于周期性发布 chip_frame -> camera_frame 的定时器
    rclcpp::TimerBase::SharedPtr camera_static_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFNode>());
    rclcpp::shutdown();
    return 0;
}

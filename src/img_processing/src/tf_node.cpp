#include "prepare_algorithm.h"
#include "serial_driver_interfaces/msg/serial_driver.hpp" // 串口消息 发送方 的 话题类型 为 [serial_driver] 类
#include "serial_driver_interfaces/msg/receive_data.hpp" // 串口消息 接收方 的 话题类型 为 [receive_data] 类
#include "serial_driver_interfaces/msg/send_pnp_info.hpp" // pnp消息 接收方 的 话题类型 为 [send_pnp_info] 类


class TFNode : public rclcpp::Node
{
public:
    TFNode(): Node("tf_node")
    {
        // 订阅当前芯片位姿
        chip_sub_ = this->create_subscription<serial_driver_interfaces::msg::ReceiveData>("/receive_data", 10, std::bind(&TFNode::ChipCallback, this, std::placeholders::_1));


        // 订阅 PnP 
        pnp_sub_ = this->create_subscription<serial_driver_interfaces::msg::SendPNPInfo>("/send_pnp_info", 10, std::bind(&TFNode::PNPCallback, this, std::placeholders::_1));


        // 初始化动态TF广播器（必须！否则空指针）
        chip_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        armorplate_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // 发布静态变换
        publishStaticCameraTransform();

        // 创建缓存对象
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        // 创建监听器，绑定缓存对象
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        RCLCPP_INFO(this->get_logger(), "静态 TF 已发布");

        // 发布串口消息
        pub_ = this->create_publisher<serial_driver_interfaces::msg::SerialDriver>("/serial_driver", 10);
    }


    // 得到【世界坐标系 -> 装甲板坐标系】的转换，并发送串口消息
    void sendSerialDriverMessage()
    {
        geometry_msgs::msg::TransformStamped transform;
        try 
        {
            transform = buffer_->lookupTransform("world_frame", "armorplate_frame", tf2::TimePointZero);
        } 
        catch (tf2::TransformException &ex) 
        {
            RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        // 获得平移向量
        double x, y, z; 
        x = transform.transform.translation.x;
        y = transform.transform.translation.y;
        z = transform.transform.translation.z;

        // 四元数 -> 欧拉角
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);

        // 获得欧拉角
        double roll, pitch, yaw;
        tf2::Matrix3x3 rpy(q);
        rpy.getRPY(roll, pitch, yaw);  // 输出 roll, pitch, yaw，单位弧

        // 弧度 -> 度
        roll = roll * 180.0 / CV_PI;
        pitch = pitch * 180.0 / CV_PI;
        yaw = yaw * 180.0 / CV_PI;



        //////////////////////// 发送串口消息 ////////////////////////

        auto serial_driver = serial_driver_interfaces::msg::SerialDriver();

        // 此时肯定有装甲板，有有效信息，直接发送
        serial_driver.roll = roll; // 发送 置信度max 装甲板的 roll
        serial_driver.pitch = pitch; // 发送 置信度max 装甲板的 pitch
        serial_driver.yaw = yaw; // 发送 置信度max 的装甲板的 yaw
        
        pub_->publish(serial_driver); // 发布消息 到 serial_driver 话题
        
        RCLCPP_INFO(this->get_logger(), "^^^^^^^^^^^^  tf_node 发布了消息: roll=%.2f pitch=%.2f yaw=%.2f", serial_driver.roll, serial_driver.pitch, serial_driver.yaw);

    }

private:

    // 【芯片坐标系 -> 相机坐标系】当前相机位姿的坐标系 -> 静态，和芯片坐标系可以视为刚体，用 t 向量
    void publishStaticCameraTransform()
    {
        static tf2_ros::StaticTransformBroadcaster static_broadcaster(this);
        geometry_msgs::msg::TransformStamped static_transform;

        // 静态变换
        static_transform.header.stamp = this->now(); //只发布一次，不在意帧头
        static_transform.header.frame_id = "chip_frame"; // 父坐标系 -> 芯片坐标系
        static_transform.child_frame_id = "camera_frame"; // 子坐标系 -> 相机坐标系

        // 平移：芯片坐标系下，相机中心位于芯片坐标系的 前55mm、下30mm
        static_transform.transform.translation.x = 0.055;  // 米
        static_transform.transform.translation.y = 0.00;
        static_transform.transform.translation.z = -0.03;    // 米

        // 无旋转，所以用单位四元数
        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;

        static_broadcaster.sendTransform(static_transform);

        RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>> Static transform chip_frame -> camera_frame published");
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
        

        // 欧拉角转四元数
        tf2::Quaternion q;
        q.setRPY(msg.roll, msg.pitch, msg.yaw);  // 顺序：roll, pitch, yaw （XYZ） 

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

        // 2. msg 中的轴角 -> cv::Mat
        cv::Mat rvec(3, 1, CV_64F);
        rvec.at<double>(0) = msg.rvec[0];
        rvec.at<double>(1) = msg.rvec[1];
        rvec.at<double>(2) = msg.rvec[2];

        // 3. 罗德里格斯变换，把 旋转向量 r -> 旋转矩阵 R
        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv); 

        // 4. 转存到 Eigen 矩阵，方便转换成四元数
        Eigen::Matrix3d R_eigen;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R_eigen(i, j) = R_cv.at<double>(i, j);
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
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFNode>());
    rclcpp::shutdown();
    return 0;
}

/*
  安装依赖
  sudo apt install ros-humble-serial-driver
  以下仅供参考
*/

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <cstring>   // 用于 memcpy
#include <algorithm> // 用于 std::copy
#include <vector>
#include <deque>     // 用于接收缓冲区

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

#include "serial_driver_interfaces/msg/serial_driver.hpp"


// 添加信号处理器，打印错误堆栈
#include <signal.h>
#include <execinfo.h>
#include <unistd.h>

void signal_handler(int sig) 
{
    void *array[20];
    size_t size = backtrace(array, 20);
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}


namespace rm_serial_driver
{

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO_ONCE(get_logger(), "Start SerialDriver!");

  RCLCPP_INFO_ONCE(get_logger(), "Step 0/10: 构造函数注册信号处理器");
  signal(SIGSEGV, signal_handler);
  signal(SIGABRT, signal_handler);
  signal(SIGFPE, signal_handler);

  RCLCPP_INFO_ONCE(get_logger(), "Step 1/10: ready to getParams()");

  getParams();
  RCLCPP_INFO_ONCE(get_logger(), "Step 2/10: getParams done, device_name=%s", device_name_.c_str());

  receive_data_pub_ = this->create_publisher<serial_driver_interfaces::msg::ReceiveData>("/receive_data", 10); // 把接收到的数据发布到 /receive_data 话题上
  RCLCPP_INFO(this->get_logger(), "Step 3/10: Created publisher for topic '/receive_data'");

  try {

    RCLCPP_INFO_ONCE(get_logger(), "Step 4/10: ready to init_port...");

    serial_driver_->init_port(device_name_, *device_config_);
    RCLCPP_INFO_ONCE(get_logger(), "Step 5/10: init_port success");

    if (!serial_driver_->port()->is_open()) {
      RCLCPP_INFO_ONCE(get_logger(), "Step 6/10: ready to opening port...");

      serial_driver_->port()->open();
      RCLCPP_INFO_ONCE(get_logger(), "Step 7/10: port opened");

      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this); // 【修改】现在启动接收线程看看
      RCLCPP_INFO_ONCE(get_logger(), "Step 8/10: receive thread started");

    }
  } 
  catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());

    RCLCPP_ERROR(get_logger(), "【串口初始化异常】串口设备不存在（/dev/ttyACM0 未连接）或无法打开。。。。。。。。。。");
    // RCLCPP_ERROR(get_logger(), "即将抛出 ex, 串口节点将退出。。。。。。。。。。。");
    
      // throw ex; // 如果要仅仅测试sendData函数，请注释掉 throw ex这一行，允许程序继续运行，并在sendData函数中仅仅打开test那一行
                // 否则节点会直接退出
     
  }

  RCLCPP_INFO_ONCE(get_logger(), "Step 9/10: ready to creating subscription");
  //Create Subscription
  target_sub_ = this->create_subscription<serial_driver_interfaces::msg::SerialDriver>(
    "/serial_driver", 10,
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
  RCLCPP_INFO_ONCE(get_logger(), "Step 10/10: subscription created");

  RCLCPP_INFO_ONCE(get_logger(), ">>>>>>>>>>>>>>> 串口构造函数已经初始化完成。");

}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}


// 自定义接收数据处理函数，发送数据到话题 receive_data 下
void RMSerialDriver::processReceivedPacket(const ReceivePacket& packet) 
{
    // 1. 先打印接收到的数据
    RCLCPP_INFO(get_logger(), "++++++++++++ Received: roll=%.2f pitch=%.2f yaw=%.2f", packet.roll, packet.pitch, packet.yaw);

    // 2. 创建并填充新消息
    auto pub_msg = serial_driver_interfaces::msg::ReceiveData();
    pub_msg.header.stamp = this->now(); // 帧头获取当前时刻
    pub_msg.roll = packet.roll;      // 翻滚角
    pub_msg.pitch = packet.pitch;    // 俯仰角
    pub_msg.yaw = packet.yaw;        // 航向角
    

    // 3. 发布新消息
    receive_data_pub_->publish(pub_msg);
    RCLCPP_INFO(get_logger(), "&&&&&&&&&&&& Published ReceiveData: roll=%.2f, pitch=%.2f, yaw=%d",
                pub_msg.roll, pub_msg.pitch, pub_msg.yaw);
}



void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) 
  {
    try 
    {
      // 读取帧头
      serial_driver_->port()->receive(header);

      if (header[0] == 0xFF) 
      {
        // 读取剩余 13 字节 -> roll picth yaw 0xFE
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        // 将帧头插入开头，组装成完整数据包
        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);

        // 校验帧尾
        if (packet.checksum == 0xFE) {

          /*
            接收数据部分
          */

          // 数据有效，进行处理（自定义函数）
          processReceivedPacket(packet);

        } else {
          RCLCPP_ERROR(get_logger(), "header[13]!=0xFE, 0xFE error!");
        }
      } 

      else 
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "header[0]!=0xFF, Invalid header: %02X", header[0]);
        continue;
      }
    } 
      catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}




void RMSerialDriver::sendData(const serial_driver_interfaces::msg::SerialDriver::SharedPtr msg)
{
    // 检查设备是否已经打开
    if (!serial_driver_->port()->is_open()) 
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10, "串口未打开，数据未发送");
        return;
    }


  try {
    SendPacket packet;
    packet.header = 0xFF; // 填充帧头

    packet.roll = msg->roll; //填充 roll
    packet.yaw = msg->yaw; // 填充 yaw
    packet.pitch = msg->pitch; // 填充 pitch
    
    packet.crc = 0xFE; //直接固定帧尾
    
    std::vector<uint8_t> data = toVector(packet);

    RCLCPP_INFO(get_logger(), "-------------READY--------------> 串口 sendData 准备发送数据, sizeof = %d", sizeof(SendPacket));
    serial_driver_->port()->send(data);

    RCLCPP_INFO(get_logger(), "-------------SUCCESSED--------------> 串口已经发送数据 roll=%.2f pitch=%.2f yaw=%.2f", msg->roll, msg->pitch, msg->yaw);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    
    // 必须：仅在设备可用时重试，避免无设备时无限重试（会直接导致串口崩溃）
    if (serial_driver_->port()->is_open()) reopenPort(); //【修改】启动 reopenPort
  }


  // RCLCPP_INFO(get_logger(), "-------------TEST--------------> 串口已经发送数据 yaw=%.2f pitch=%.2f", msg->yaw, msg->pitch);
}



void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");
    RCLCPP_INFO(get_logger(), "flow_control string = '%s'", fc_string.c_str());

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}


}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)

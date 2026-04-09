// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0xFF;
  
  // --- 有效数据起 ---
  float yaw;      // 先 yaw
  float pitch;      // 然后 pitch
  float roll;    // 最后 roll
  // --- 有效数据止 ---

  uint8_t checksum = 0xFE; // CRC8 校验值，注意这里是固定
} __attribute__((packed));


// 【修改】头尾代码 pragma作用：强制结构体对齐
#pragma pack(push, 1)
struct SendPacket
{
  uint8_t header = 0xFF;
  
  // --- 有效数据起 ---
  float yaw;      // 先 yaw
  float pitch;    // 后 pitch
  // --- 有效数据止 ---

  uint8_t crc = 0XFE;   // CRC8 校验值，注意这里是固定
};
#pragma pack(pop)



inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}



}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_

// Unit tests for rm_serial_driver packet serialization
// Tests: fromVector, toVector, struct layout

#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <vector>

#include "rm_serial_driver/packet.hpp"

class PacketTest : public ::testing::Test {};

// --- SendPacket struct layout ---

TEST_F(PacketTest, SendPacketSizeIsPacked)
{
    // SendPacket: 1 byte header + 4 bytes pitch + 4 bytes yaw + 1 byte crc = 10 bytes
    EXPECT_EQ(sizeof(rm_serial_driver::SendPacket), 10u);
}

TEST_F(PacketTest, ReceivePacketSizeIsPacked)
{
    // ReceivePacket: 1 byte header + 4*3 floats + 1 byte checksum = 14 bytes
    EXPECT_EQ(sizeof(rm_serial_driver::ReceivePacket), 14u);
}

// --- toVector ---

TEST_F(PacketTest, ToVectorProducesCorrectSize)
{
    rm_serial_driver::SendPacket pkt;
    pkt.header = 0xFF;
    pkt.absolute_pitch = 1.0f;
    pkt.absolute_yaw = 2.0f;
    pkt.crc = 0xFE;

    std::vector<uint8_t> vec = rm_serial_driver::toVector(pkt);
    EXPECT_EQ(vec.size(), sizeof(rm_serial_driver::SendPacket));
}

TEST_F(PacketTest, ToVectorHeaderAndCRC)
{
    rm_serial_driver::SendPacket pkt;
    pkt.header = 0xFF;
    pkt.absolute_pitch = 0.0f;
    pkt.absolute_yaw = 0.0f;
    pkt.crc = 0xFE;

    std::vector<uint8_t> vec = rm_serial_driver::toVector(pkt);
    EXPECT_EQ(vec[0], 0xFF);
    EXPECT_EQ(vec[vec.size() - 1], 0xFE);
}

TEST_F(PacketTest, ToVectorPreservesFloatData)
{
    rm_serial_driver::SendPacket pkt;
    pkt.header = 0xFF;
    pkt.absolute_pitch = 3.14f;
    pkt.absolute_yaw = -1.57f;
    pkt.crc = 0xFE;

    std::vector<uint8_t> vec = rm_serial_driver::toVector(pkt);

    // Reconstruct the packet from raw bytes
    rm_serial_driver::SendPacket reconstructed;
    std::memcpy(&reconstructed, vec.data(), sizeof(rm_serial_driver::SendPacket));

    EXPECT_FLOAT_EQ(reconstructed.absolute_pitch, 3.14f);
    EXPECT_FLOAT_EQ(reconstructed.absolute_yaw, -1.57f);
}

// --- fromVector ---

TEST_F(PacketTest, FromVectorReconstructsHeader)
{
    rm_serial_driver::ReceivePacket original;
    original.header = 0xFF;
    original.euler_pitch = 10.0f;
    original.euler_yaw = 20.0f;
    original.euler_roll = 30.0f;
    original.checksum = 0xFE;

    // Serialize manually
    std::vector<uint8_t> data(sizeof(rm_serial_driver::ReceivePacket));
    std::memcpy(data.data(), &original, sizeof(rm_serial_driver::ReceivePacket));

    rm_serial_driver::ReceivePacket result = rm_serial_driver::fromVector(data);
    EXPECT_EQ(result.header, 0xFF);
    EXPECT_EQ(result.checksum, 0xFE);
}

TEST_F(PacketTest, FromVectorReconstructsEulerAngles)
{
    rm_serial_driver::ReceivePacket original;
    original.header = 0xFF;
    original.euler_pitch = 45.5f;
    original.euler_yaw = -30.2f;
    original.euler_roll = 15.7f;
    original.checksum = 0xFE;

    std::vector<uint8_t> data(sizeof(rm_serial_driver::ReceivePacket));
    std::memcpy(data.data(), &original, sizeof(rm_serial_driver::ReceivePacket));

    rm_serial_driver::ReceivePacket result = rm_serial_driver::fromVector(data);
    EXPECT_FLOAT_EQ(result.euler_pitch, 45.5f);
    EXPECT_FLOAT_EQ(result.euler_yaw, -30.2f);
    EXPECT_FLOAT_EQ(result.euler_roll, 15.7f);
}

// --- Round-trip consistency ---

TEST_F(PacketTest, SendPacketRoundTrip)
{
    rm_serial_driver::SendPacket original;
    original.header = 0xFF;
    original.absolute_pitch = 12.345f;
    original.absolute_yaw = -67.89f;
    original.crc = 0xFE;

    std::vector<uint8_t> vec = rm_serial_driver::toVector(original);

    // Reconstruct
    rm_serial_driver::SendPacket decoded;
    std::memcpy(&decoded, vec.data(), vec.size());

    EXPECT_EQ(decoded.header, original.header);
    EXPECT_FLOAT_EQ(decoded.absolute_pitch, original.absolute_pitch);
    EXPECT_FLOAT_EQ(decoded.absolute_yaw, original.absolute_yaw);
    EXPECT_EQ(decoded.crc, original.crc);
}

TEST_F(PacketTest, ReceivePacketRoundTrip)
{
    rm_serial_driver::ReceivePacket original;
    original.header = 0xFF;
    original.euler_pitch = 0.123f;
    original.euler_yaw = -0.456f;
    original.euler_roll = 0.789f;
    original.checksum = 0xFE;

    std::vector<uint8_t> data(sizeof(rm_serial_driver::ReceivePacket));
    std::memcpy(data.data(), &original, sizeof(rm_serial_driver::ReceivePacket));

    rm_serial_driver::ReceivePacket decoded = rm_serial_driver::fromVector(data);

    EXPECT_EQ(decoded.header, original.header);
    EXPECT_FLOAT_EQ(decoded.euler_pitch, original.euler_pitch);
    EXPECT_FLOAT_EQ(decoded.euler_yaw, original.euler_yaw);
    EXPECT_FLOAT_EQ(decoded.euler_roll, original.euler_roll);
    EXPECT_EQ(decoded.checksum, original.checksum);
}

// --- Edge cases ---

TEST_F(PacketTest, SendPacketZeroValues)
{
    rm_serial_driver::SendPacket pkt;
    pkt.header = 0xFF;
    pkt.absolute_pitch = 0.0f;
    pkt.absolute_yaw = 0.0f;
    pkt.crc = 0xFE;

    std::vector<uint8_t> vec = rm_serial_driver::toVector(pkt);
    EXPECT_EQ(vec.size(), 10u);

    rm_serial_driver::SendPacket decoded;
    std::memcpy(&decoded, vec.data(), vec.size());
    EXPECT_FLOAT_EQ(decoded.absolute_pitch, 0.0f);
    EXPECT_FLOAT_EQ(decoded.absolute_yaw, 0.0f);
}

TEST_F(PacketTest, SendPacketExtremeValues)
{
    rm_serial_driver::SendPacket pkt;
    pkt.header = 0xFF;
    pkt.absolute_pitch = 1e10f;
    pkt.absolute_yaw = -1e10f;
    pkt.crc = 0xFE;

    std::vector<uint8_t> vec = rm_serial_driver::toVector(pkt);
    rm_serial_driver::SendPacket decoded;
    std::memcpy(&decoded, vec.data(), vec.size());

    EXPECT_FLOAT_EQ(decoded.absolute_pitch, 1e10f);
    EXPECT_FLOAT_EQ(decoded.absolute_yaw, -1e10f);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

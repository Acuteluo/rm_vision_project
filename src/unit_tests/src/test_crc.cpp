// Unit tests for rm_serial_driver CRC16 functions
// Tests: Get_CRC16_Check_Sum, Verify_CRC16_Check_Sum, Append_CRC16_Check_Sum

#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>

#include "rm_serial_driver/crc.hpp"

// Get_CRC16_Check_Sum is defined in crc.cpp but not exposed in crc.hpp.
// Declare it here for direct testing.
namespace crc16 {
uint16_t Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC);
}  // namespace crc16

class CRC16Test : public ::testing::Test {};

// --- Get_CRC16_Check_Sum ---

TEST_F(CRC16Test, NullPointerReturnsInitValue)
{
    uint16_t result = crc16::Get_CRC16_Check_Sum(nullptr, 10, 0xFFFF);
    EXPECT_EQ(result, 0xFFFF);
}

TEST_F(CRC16Test, ZeroLengthReturnsInit)
{
    uint8_t data[] = {0x01, 0x02};
    uint16_t result = crc16::Get_CRC16_Check_Sum(data, 0, 0xFFFF);
    EXPECT_EQ(result, 0xFFFF);
}

TEST_F(CRC16Test, SingleByteZero)
{
    uint8_t data[] = {0x00};
    uint16_t result = crc16::Get_CRC16_Check_Sum(data, 1, 0xFFFF);
    // CRC16 of single 0x00 with init 0xFFFF using this table
    EXPECT_NE(result, 0xFFFF); // Should change from init
}

TEST_F(CRC16Test, KnownSequence)
{
    // Test with a known data pattern
    uint8_t data[] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint16_t crc = crc16::Get_CRC16_Check_Sum(data, 9, 0xFFFF);
    // Verify determinism: same input produces same output
    uint16_t crc2 = crc16::Get_CRC16_Check_Sum(data, 9, 0xFFFF);
    EXPECT_EQ(crc, crc2);
}

TEST_F(CRC16Test, DifferentDataProducesDifferentCRC)
{
    uint8_t data1[] = {0x01, 0x02, 0x03};
    uint8_t data2[] = {0x04, 0x05, 0x06};
    uint16_t crc1 = crc16::Get_CRC16_Check_Sum(data1, 3, 0xFFFF);
    uint16_t crc2 = crc16::Get_CRC16_Check_Sum(data2, 3, 0xFFFF);
    EXPECT_NE(crc1, crc2);
}

TEST_F(CRC16Test, DifferentInitProducesDifferentCRC)
{
    uint8_t data[] = {0x01, 0x02, 0x03};
    uint16_t crc1 = crc16::Get_CRC16_Check_Sum(data, 3, 0xFFFF);
    uint16_t crc2 = crc16::Get_CRC16_Check_Sum(data, 3, 0x0000);
    EXPECT_NE(crc1, crc2);
}

// --- Verify_CRC16_Check_Sum ---

TEST_F(CRC16Test, VerifyNullPointerReturnsFalse)
{
    uint32_t result = crc16::Verify_CRC16_Check_Sum(nullptr, 10);
    EXPECT_EQ(result, 0u);
}

TEST_F(CRC16Test, VerifyTooShortReturnsFalse)
{
    uint8_t data[] = {0x01};
    uint32_t result = crc16::Verify_CRC16_Check_Sum(data, 1);
    EXPECT_EQ(result, 0u);

    uint32_t result2 = crc16::Verify_CRC16_Check_Sum(data, 2);
    EXPECT_EQ(result2, 0u);
}

TEST_F(CRC16Test, VerifyAfterAppendReturnsTrue)
{
    // Create a buffer with data + 2 bytes for CRC
    uint8_t buffer[10] = {0xFF, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x00};
    uint32_t total_len = 10;

    // Append CRC to last 2 bytes
    crc16::Append_CRC16_Check_Sum(buffer, total_len);

    // Verify should pass
    uint32_t valid = crc16::Verify_CRC16_Check_Sum(buffer, total_len);
    EXPECT_NE(valid, 0u);
}

TEST_F(CRC16Test, VerifyCorruptedDataReturnsFalse)
{
    uint8_t buffer[10] = {0xFF, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x00};
    uint32_t total_len = 10;

    crc16::Append_CRC16_Check_Sum(buffer, total_len);

    // Corrupt one data byte
    buffer[3] = 0xFF;

    uint32_t valid = crc16::Verify_CRC16_Check_Sum(buffer, total_len);
    EXPECT_EQ(valid, 0u);
}

// --- Append_CRC16_Check_Sum ---

TEST_F(CRC16Test, AppendNullPointerDoesNotCrash)
{
    // Should not crash
    crc16::Append_CRC16_Check_Sum(nullptr, 10);
}

TEST_F(CRC16Test, AppendTooShortDoesNotCrash)
{
    uint8_t data[] = {0x01};
    crc16::Append_CRC16_Check_Sum(data, 1);
    // data should remain unchanged
    EXPECT_EQ(data[0], 0x01);
}

TEST_F(CRC16Test, AppendWritesCRCToLastTwoBytes)
{
    uint8_t buffer[5] = {0xAA, 0xBB, 0xCC, 0x00, 0x00};
    crc16::Append_CRC16_Check_Sum(buffer, 5);

    // Last two bytes should now contain the CRC
    // At least one of them should be non-zero for non-trivial data
    EXPECT_TRUE(buffer[3] != 0x00 || buffer[4] != 0x00);
}

TEST_F(CRC16Test, AppendAndVerifyConsistency)
{
    // Test with various data patterns
    for (uint8_t pattern = 0; pattern < 10; ++pattern) {
        uint8_t buffer[6];
        buffer[0] = 0xFF;           // header
        buffer[1] = pattern;
        buffer[2] = pattern + 1;
        buffer[3] = pattern + 2;
        buffer[4] = 0x00;           // CRC placeholder
        buffer[5] = 0x00;           // CRC placeholder

        crc16::Append_CRC16_Check_Sum(buffer, 6);
        uint32_t valid = crc16::Verify_CRC16_Check_Sum(buffer, 6);
        EXPECT_NE(valid, 0u) << "Failed for pattern=" << (int)pattern;
    }
}

TEST_F(CRC16Test, CRCChangesWhenDataChanges)
{
    uint8_t buf1[5] = {0x01, 0x02, 0x03, 0x00, 0x00};
    uint8_t buf2[5] = {0x01, 0x02, 0x04, 0x00, 0x00};

    crc16::Append_CRC16_Check_Sum(buf1, 5);
    crc16::Append_CRC16_Check_Sum(buf2, 5);

    // CRC bytes should differ since data differs
    EXPECT_TRUE(buf1[3] != buf2[3] || buf1[4] != buf2[4]);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

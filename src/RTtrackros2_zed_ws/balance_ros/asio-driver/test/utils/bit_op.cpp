#include "asio_driver/utils/bit_op.hpp"
#include "gtest/gtest.h"

TEST(bit_op, byte_set)
{
    unsigned char byte = 0xff;
    AD::Utils::BitOp::SetBit(byte, 7, false);
    EXPECT_EQ(byte, 0x7f);
    AD::Utils::BitOp::SetBits(byte, 0, 3, 0);
    EXPECT_EQ(byte, 0x70);
}

TEST(bit_op, byte_get)
{
    unsigned char byte = 0x72;
    EXPECT_EQ(AD::Utils::BitOp::GetBit(byte, 7), false);
    EXPECT_EQ(AD::Utils::BitOp::GetBits(byte, 0, 3), 0x2);
}

TEST(bit_op, buffer_set)
{
    unsigned char buffer[4] = {0xff, 0xff, 0x00, 0x00};
    AD::Utils::BitOp::SetBitOnBuffer(buffer, 1, false);
    EXPECT_EQ(buffer[0], 0xfd);
    AD::Utils::BitOp::SetBitsOnBuffer(buffer, 12, 15, 0x0);
    EXPECT_EQ(buffer[1], 0x0f);
    AD::Utils::BitOp::SetBitsOnBuffer(buffer, 20, 27, 0x85);
    EXPECT_EQ(buffer[2], 0x80);
    EXPECT_EQ(buffer[3], 0x05);
}

TEST(bit_op, buffer_get)
{
    unsigned char buffer[4] = {0xff, 0xff, 0x12, 0x34};
    bool bit = AD::Utils::BitOp::GetBitOnBuffer(buffer, 16);
    EXPECT_EQ(bit, false);
    bit = AD::Utils::BitOp::GetBitOnBuffer(buffer, 15);
    EXPECT_EQ(bit, true);
    unsigned long ret = AD::Utils::BitOp::GetBitsOnBuffer(buffer, 12, 27);
    EXPECT_EQ(ret, 0xf124);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#include "asio_driver/utils/bit_coder.hpp"
#include "gtest/gtest.h"

TEST(bit_coder_bigendian, bytes_set)
{
    // 0x12345678
    unsigned char data[] = {0x12, 0x34, 0x56, 0x78};
    AD::Utils::BitCoder coder(4, AD::Utils::BitCoder::ByteOrder::BigEndian, data);
    auto span = coder.AsSpan();
    coder.SetBit(1, false);
    EXPECT_EQ(span[0], 0x10);

    coder.SetBits(1, 2, 0b11);
    EXPECT_EQ(span[0], 0x16);

    coder.SetBits(12, 19, 0xff);
    EXPECT_EQ(span[1], 0xf4);
    EXPECT_EQ(span[2], 0x5f);
}

TEST(bit_coder_bigendian, bytes_get) {}

TEST(bit_coder_littleendian, bytes_set) {}

TEST(bit_coder_littleendian, bytes_get) {}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#include "asio_driver/utils/bit_op.hpp"
#include <cassert>

namespace AD::Utils::BitOp
{
void SetBit(unsigned char &byte, size_t bit, bool value)
{
    assert(bit < 8);
    if (value)
        byte |= (1 << bit);
    else
        byte &= ~(1 << bit);
}

void SetBits(unsigned char &byte, size_t start, size_t end, unsigned char value)
{
    assert(start < 8 && end < 8 && start <= end);
    unsigned char mask = 0xff;
    mask >>= start;
    mask <<= start + 7 - end;
    mask >>= 7 - end;
    byte &= ~mask;
    byte |= (value << start);
}

bool GetBit(unsigned char byte, size_t bit)
{
    return byte & (1 << bit);
}

unsigned char GetBits(unsigned char byte, size_t start, size_t end)
{
    assert(start < 8 && end < 8 && start <= end);
    return (byte >> start) & ((1 << (end - start + 1)) - 1);
}

void SetBitOnBuffer(std::span<unsigned char> buffer, size_t bit, bool value)
{
    size_t byteIndex = bit / 8;
    size_t bitIndex = bit % 8;
    assert(byteIndex < buffer.size_bytes());
    SetBit(buffer[byteIndex], bitIndex, value);
}

bool GetBitOnBuffer(std::span<unsigned char> buffer, size_t bit)
{
    size_t byteIndex = bit / 8;
    size_t bitIndex = bit % 8;
    assert(byteIndex < buffer.size_bytes());
    return GetBit(buffer[byteIndex], bitIndex);
}

void SetBitsOnBuffer(std::span<unsigned char> buffer, size_t start, size_t end, u_int64_t value)
{
    size_t byteIndexStart = start / 8;
    size_t bitIndexStart = start % 8;
    size_t byteIndexEnd = end / 8;
    size_t bitIndexEnd = end % 8;
    assert(byteIndexEnd < buffer.size_bytes() && byteIndexStart);
    if (byteIndexStart == byteIndexEnd)
    {
        SetBits(buffer[byteIndexStart], bitIndexStart, bitIndexEnd, value);
        return;
    }
    else
    {
        // first byte input value high bits
        SetBits(buffer[byteIndexEnd], 0, bitIndexEnd, value & (0xff >> (bitIndexEnd + 1)));
        for (size_t i = byteIndexEnd - 1; i > byteIndexStart; i--)
        {
            value >>= 8;
            buffer[i] = value & 0xff;
        }
        value >>= (8 - bitIndexStart);
        SetBits(buffer[byteIndexStart], bitIndexStart, 7, value & (0xff >> (8 - bitIndexStart)));
    }
}

u_int64_t GetBitsOnBuffer(std::span<unsigned char> buffer, size_t start, size_t end)
{
    size_t byteIndexStart = start / 8;
    size_t bitIndexStart = start % 8;
    size_t byteIndexEnd = end / 8;
    size_t bitIndexEnd = end % 8;
    assert(byteIndexEnd < buffer.size_bytes() && byteIndexStart < buffer.size_bytes());
    if (byteIndexStart == byteIndexEnd)
    {
        return GetBits(buffer[byteIndexStart], bitIndexStart, bitIndexEnd);
    }
    else
    {
        u_int64_t value = 0;
        value |= GetBits(buffer[byteIndexStart], bitIndexStart, 7);
        for (size_t i = byteIndexStart + 1; i < byteIndexEnd; i++)
        {
            value <<= 8;
            value |= buffer[i];
        }
        value <<= bitIndexEnd + 1;
        value |= GetBits(buffer[byteIndexEnd], 0, bitIndexEnd);
        return value;
    }
}
} // namespace AD::Utils::BitOp
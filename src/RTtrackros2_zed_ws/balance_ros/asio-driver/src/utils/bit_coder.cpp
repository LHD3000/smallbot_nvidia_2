#include "asio_driver/utils/bit_coder.hpp"
#include <cassert>
#include <cstring>
#include <memory>

namespace AD::Utils
{
BitCoder::BitCoder(ssize_t size, ByteOrder order, unsigned char *data)
    : size(size), order(order)
{
    buffer = std::make_unique<unsigned char[]>(size);
    if (data != nullptr)
    {
        memcpy(buffer.get(), data, size);
    }
    else
    {
        memset(buffer.get(), 0, size);
    }
}

unsigned char &BitCoder::operator[](size_t index)
{
    return buffer[index];
}

std::span<const unsigned char> BitCoder::AsSpan()
{
    return std::span<const unsigned char>(buffer.get(), size);
}

unsigned char *BitCoder::data()
{
    return buffer.get();
}
ssize_t BitCoder::Size()
{
    return size;
}
void BitCoder::Reset()
{
    memset(buffer.get(), 0, size);
}
void BitCoder::SetBit(size_t bit, bool value)
{
    size_t byteIndex = bit / 8;
    size_t bitIndex = bit % 8;
    assert(byteIndex < size);
    SetBit(buffer[byteIndex], bitIndex, value);
}
bool BitCoder::GetBit(size_t bit)
{
    size_t byteIndex = bit / 8;
    size_t bitIndex = bit % 8;
    assert(byteIndex < size);
    return GetBit(buffer[byteIndex], bitIndex);
}

void BitCoder::SetBits(size_t start, size_t end, u_int64_t value)
{
    size_t byteIndexStart = start / 8;
    size_t bitIndexStart = start % 8;
    size_t byteIndexEnd = end / 8;
    size_t bitIndexEnd = end % 8;
    assert(byteIndexEnd < size && byteIndexStart < size);
    if (byteIndexStart == byteIndexEnd)
    {
        SetBits(buffer[byteIndexStart], bitIndexStart, bitIndexEnd, value);
        return;
    }
    else
    {
        if (order == ByteOrder::BigEndian)
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
        else // order == ByteOrder::LittleEndian
        {
            SetBits(buffer[byteIndexStart], bitIndexStart, 7, value & (0xff >> (8 - bitIndexStart)));
            for (size_t i = byteIndexEnd - 1; i > byteIndexStart; i--)
            {
                value >>= 8;
                buffer[i] = value & 0xff;
            }
            value >>= bitIndexEnd + 1;
            SetBits(buffer[byteIndexEnd], 0, bitIndexEnd, value & (0xff >> (bitIndexEnd + 1)));
        }
    }
}
u_int64_t BitCoder::GetBits(size_t start, size_t end)
{
    size_t byteIndexStart = start / 8;
    size_t bitIndexStart = start % 8;
    size_t byteIndexEnd = end / 8;
    size_t bitIndexEnd = end % 8;
    assert(byteIndexEnd < size && byteIndexStart < size);
    if (byteIndexStart == byteIndexEnd)
    {
        return GetBits(buffer[byteIndexStart], bitIndexStart, bitIndexEnd);
    }
    else
    {
        if (order == ByteOrder::BigEndian)
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
        else // order == ByteOrder::LittleEndian
        {
            u_int64_t value = 0;
            value |= GetBits(buffer[byteIndexEnd], 0, bitIndexEnd);
            for (size_t i = byteIndexEnd - 1; i > byteIndexStart; i--)
            {
                value <<= 8;
                value |= buffer[i];
            }
            value <<= 8 - bitIndexStart;
            value |= GetBits(buffer[byteIndexStart], bitIndexStart, 7);
            return value;
        }
    }
}

void BitCoder::SetBit(unsigned char &byte, size_t bit, bool value)
{
    assert(bit < 8);
    if (value)
        byte |= (1 << bit);
    else
        byte &= ~(1 << bit);
}
void BitCoder::SetBits(unsigned char &byte, size_t start, size_t end, unsigned char value)
{
    assert(start < 8 && end < 8 && start <= end);
    unsigned char mask = 0xff;
    mask >>= start;
    mask <<= start + 7 - end;
    mask >>= 7 - end;
    byte &= ~mask;
    byte |= (value << start);
}
bool BitCoder::GetBit(unsigned char byte, size_t bit)
{
    return byte & (1 << bit);
}
unsigned char BitCoder::GetBits(unsigned char byte, size_t start, size_t end)
{
    assert(start < 8 && end < 8 && start <= end);
    return (byte >> start) & ((1 << (end - start + 1)) - 1);
}
} // namespace AD::Utils
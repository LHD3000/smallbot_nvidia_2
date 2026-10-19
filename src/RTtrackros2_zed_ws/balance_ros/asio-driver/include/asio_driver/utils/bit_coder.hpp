#pragma once
#include <memory>
#include <span>
#include <sys/types.h>

namespace AD::Utils
{

// TODO check value is in range on set/sets
class BitCoder
{
  public:
    enum class ByteOrder
    {
        LittleEndian,
        BigEndian,
    };

  private:
    ssize_t size;
    ByteOrder order;
    std::unique_ptr<unsigned char[]> buffer;

  public:
    /*
     * if data set nullptr,coder will locate memory,otherwise will copy data;
     */
    BitCoder(ssize_t bytes, ByteOrder order, unsigned char *data = nullptr);
    unsigned char &operator[](size_t index);
    std::span<const unsigned char> AsSpan();
    unsigned char *data();
    ssize_t Size();
    void Reset();
    void SetBit(size_t bit, bool value);
    bool GetBit(size_t bit);
    void SetBits(size_t start, size_t end, u_int64_t value);
    u_int64_t GetBits(size_t start, size_t end);

  private:
    void SetBit(unsigned char &byte, size_t bit, bool value);
    void SetBits(unsigned char &byte, size_t start, size_t end, unsigned char value);
    bool GetBit(unsigned char byte, size_t bit);
    unsigned char GetBits(unsigned char byte, size_t start, size_t end);
};
} // namespace AD::Utils
#pragma once
#include <cstddef>
#include <span>
#include <sys/types.h>

namespace AD::Utils::BitOp
{
void SetBit(unsigned char &byte, size_t bit, bool value);
void SetBits(unsigned char &byte, size_t start, size_t end, unsigned char value);
bool GetBit(unsigned char byte, size_t bit);
unsigned char GetBits(unsigned char byte, size_t start, size_t end);
void SetBitOnBuffer(std::span<unsigned char> buffer, size_t bit, bool value);
bool GetBitOnBuffer(std::span<unsigned char> buffer, size_t bit);
void SetBitsOnBuffer(std::span<unsigned char> buffer, size_t start, size_t end, u_int64_t value);
u_int64_t GetBitsOnBuffer(std::span<unsigned char> buffer, size_t start, size_t end);

} // namespace AD::Utils::BitOp

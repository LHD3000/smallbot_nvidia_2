#pragma once

#include "asio_driver/io/serial.hpp"
#include <deque>
#include <sstream>
#include <string>

namespace AD::Special
{

class IMUWitCmp10a
{
  public:
    struct IMUState
    {
        float acc[3]{0, 0, 0};
        float gyro[3]{0, 0, 0};
        float angle[3]{0, 0, 0};

        std::string ToString() const;
    };
    enum class DataUpdateFlag : unsigned char
    {
        None = 0x00,
        Acc = 0x01,
        Gyro = 0x02,
        Angle = 0x04,
        All = Acc | Gyro | Angle,
    };

  private:
    volatile char dataUpdateFlag = 0;
    std::unique_ptr<Serial> serial;
    AsioContext::SharePtr context;
    std::deque<unsigned char> dataQueue;
    IMUState state;
    bool isOpen = false;
    DataUpdateFlag newDataFlag = DataUpdateFlag::None;

  public:
    IMUWitCmp10a(AsioContext::SharePtr context);
    ~IMUWitCmp10a();
    bool Open(const std::string &dev, unsigned int baudrate);
    bool IsOpen() const;
    bool Close();
    DataUpdateFlag GetNewDataFlag() const;
    bool HasNewData() const;
    IMUState Read();

  private:
    void OnReceive(std::span<const unsigned char> data);
};

} // namespace AD::Special

#include "asio_driver_special/motor_brt_can.hpp"
#include <iostream>

namespace AD::Special
{
MotorBrtCan::MotorBrtCan(AsioContext::SharePtr context)
{
    this->context = context;
}
MotorBrtCan::~MotorBrtCan()
{
    if (isOpen)
    {
        Close();
    }
}

bool MotorBrtCan::Open(CanBus::SharePtr bus, const MotorConfig &cfg)
{
    this->bus = bus;
    this->cfg = cfg;
    isOpen = bus->AddListener(cfg.address, std::bind(&MotorBrtCan::OnReceive, this, std::placeholders::_1));
    return isOpen;
}
bool MotorBrtCan::Close()
{
    if (bus->RemoveLinstener(cfg.address))
    {
        isOpen = false;
        return true;
    }
    return false;
}
bool MotorBrtCan::IsOpen()
{
    return isOpen;
}

void MotorBrtCan::OnReceive(std::span<const unsigned char> data)
{
    newData = true;
    if (data[0] == 0x0f && data[1] == 0x01 && data.size() == 6)
    {
        states.erpm = data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5];
        states.rpm = (float)(states.erpm) / cfg.poleCount;
        return;
    }
    else if (data[0] == 0x0f && data[1] == 0x05 && data.size() == 4)
    {
        short raw = data[2] << 8 | data[3];
        states.current = static_cast<float>(raw) * 10.0f;
        return;
    }
    else if ((data[0] == 0x0f && data.size() == 2) || (data[0] == 0x00 && data.size() == 1) || data[0] == 0x01)
    {
        // request and heartbeat,control
    }
    else
    {
        std::cerr << "err " << data.size() << std::endl;
        for (auto c : data)
        {
            std::cerr << std::hex << (int)c << " ";
        }
    }
    newData = false;
}

bool MotorBrtCan::RequestRPM()
{
    constexpr unsigned char mes[2] = {0x0f, 0x01};
    return bus->Send(cfg.address, std::span<const unsigned char>(mes));
}

bool MotorBrtCan::RequestCurrent()
{
    constexpr unsigned char mes[2] = {0x0f, 0x05};
    return bus->Send(cfg.address, std::span<const unsigned char>(mes));
}

bool MotorBrtCan::SetRPM(float value)
{
    int32_t erpm = value * cfg.poleCount;
    unsigned char buffer[5];
    buffer[0] = 0x02;
    unsigned char *ptr = reinterpret_cast<unsigned char *>(&erpm);
    // attention: little-endian for linux
    buffer[1] = ptr[3];
    buffer[2] = ptr[2];
    buffer[3] = ptr[1];
    buffer[4] = ptr[0];
    return bus->Send(cfg.address, buffer);
}
bool MotorBrtCan::SetPWM(float precent)
{
    int16_t value = (precent * 1000);
    unsigned char buffer[3];
    buffer[0] = 0x03;
    unsigned char *ptr = reinterpret_cast<unsigned char *>(&value);
    // attention: little-endian for linux
    buffer[1] = ptr[1];
    buffer[2] = ptr[0];
    return bus->Send(cfg.address, buffer);
}
bool MotorBrtCan::SetCurrent(float mA)
{
    int16_t value = (mA / 10);
    unsigned char buffer[3];
    buffer[0] = 0x01;
    unsigned char *ptr = reinterpret_cast<unsigned char *>(&value);
    // attention: little-endian for linux
    buffer[1] = ptr[1];
    buffer[2] = ptr[0];
    return bus->Send(cfg.address, buffer);
}

bool MotorBrtCan::SendHeartBeat()
{
    constexpr unsigned char mes = 0x00;
    return bus->Send(cfg.address, std::span<const unsigned char>(&mes, 1));
}

bool MotorBrtCan::HasNewData() const
{
    return newData;
}

MotorBrtCan::MotorStates MotorBrtCan::ReadState()
{
    newData = false;
    return states;
}
} // namespace AD::Special
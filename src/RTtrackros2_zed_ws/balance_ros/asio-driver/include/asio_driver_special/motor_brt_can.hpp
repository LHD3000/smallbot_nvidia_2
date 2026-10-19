#pragma once

#include "asio_driver/bus/can_bus.hpp"

namespace AD::Special
{
class MotorBrtCan
{
  public:
#pragma regioin enumCode
    enum class ErrorCode
    {
        NoError = 0,
        OverVoltage = 1,
        UnderVoltage = 2,
        OverAbsoluteCurrent = 3,
        MOSOverTemperature = 4,
        MCUUnderVoltage = 5,
        WatchDogTriggering = 6,
        SPIInterfaceError = 7,
        FlashBroken = 8,
        UCurrentOverPhase = 9,
        VCurrentOverPhase = 10,
        WCurrentOverPhase = 11,
        CurrentInbalance = 12,
        ConfigInFLASHBroken = 13,
        ProgramInFlashBroken = 14
    };
#pragma endregion
    struct MotorConfig
    {
        Can::CanID address;
        int poleCount;
    };
    struct MotorStates
    {
        int erpm = 0;
        float rpm = 0.0f; 
        // mA
        float current = 0.0f;
    };

  private:
    CanBus::SharePtr bus;
    MotorConfig cfg;
    MotorStates states;
    AsioContext::SharePtr context;
    bool isOpen = false;
    bool newData = false;

  public:
    MotorBrtCan(AsioContext::SharePtr context);
    ~MotorBrtCan();
    bool Open(CanBus::SharePtr bus, const MotorConfig &cfg);
    bool Close();
    bool IsOpen();
    bool RequestRPM();
    bool RequestCurrent();
    bool SetRPM(float value);
    bool SetPWM(float precent);
    bool SetCurrent(float mA);
    bool SendHeartBeat();

    bool HasNewData() const;
    MotorStates ReadState();

  private:
    void OnReceive(std::span<const unsigned char> data);
};
} // namespace AD::Special
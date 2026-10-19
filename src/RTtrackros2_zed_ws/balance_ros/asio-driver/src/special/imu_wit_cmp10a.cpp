#include "asio_driver_special/imu_wit_cmp10a.hpp"
#include <iomanip>

namespace AD::Special
{
enum class DataTypeFlag : unsigned char
{
    Acc = 0x51,
    Angle = 0x53,
    Gyro = 0x52,
    Mangnetic = 0x54,
};

std::string IMUWitCmp10a::IMUState::ToString() const
{
    using namespace std;
    ostringstream info;
    info << fixed << setprecision(1);
    info << "acc:\t" << acc[0] << ",\t" << acc[1] << ",\t" << acc[2] << '\n';
    info << "gyro:\t" << gyro[0] << ",\t" << gyro[1] << ",\t" << gyro[2] << '\n';
    info << "angle:\t" << angle[0] << ",\t" << angle[1] << ",\t" << angle[2] << '\n';
    return info.str();
}

IMUWitCmp10a::IMUWitCmp10a(AsioContext::SharePtr context)
{
    this->context = context;
    serial = std::make_unique<Serial>(context);
}
IMUWitCmp10a::~IMUWitCmp10a()
{
    if (IsOpen())
    {
        Close();
    }
}
bool IMUWitCmp10a::Open(const std::string &device, unsigned int baudrate)
{
    serial->OnReceive(std::bind(&IMUWitCmp10a::OnReceive, this, std::placeholders::_1));
    isOpen = serial->Open(device, baudrate);
    return isOpen;
}
bool IMUWitCmp10a::IsOpen() const
{
    return isOpen;
}
bool IMUWitCmp10a::Close()
{
    if (serial->Close())
    {
        isOpen = false;
        return true;
    }
    return false;
}
IMUWitCmp10a::DataUpdateFlag IMUWitCmp10a::GetNewDataFlag() const
{
    return newDataFlag;
}

bool IMUWitCmp10a::HasNewData() const
{
    return newDataFlag != DataUpdateFlag::None;
}

IMUWitCmp10a::IMUState IMUWitCmp10a::Read()
{
    newDataFlag = DataUpdateFlag::None;
    return state;
}

void IMUWitCmp10a::OnReceive(std::span<const unsigned char> data)
{
    dataQueue.assign(data.begin(), data.end());
    while (!dataQueue.empty() && dataQueue[0] != 0x55)
    {
        dataQueue.pop_front();
    }
    while (dataQueue.size() >= 11)
    {
        unsigned char sum = 0;
        for (size_t i = 0; i < 10; i++)
        {
            sum += dataQueue[i];
        }
        if (sum != dataQueue[10])
        {
            dataQueue.pop_front();
            continue;
        }
        int16_t usData[4];
        usData[0] = static_cast<int16_t>(((uint16_t)dataQueue[3] << 8) | (uint16_t)dataQueue[2]);
        usData[1] = static_cast<int16_t>(((uint16_t)dataQueue[5] << 8) | (uint16_t)dataQueue[4]);
        usData[2] = static_cast<int16_t>(((uint16_t)dataQueue[7] << 8) | (uint16_t)dataQueue[6]);
        usData[3] = static_cast<int16_t>(((uint16_t)dataQueue[9] << 8) | (uint16_t)dataQueue[8]);
        DataTypeFlag flag = static_cast<DataTypeFlag>(dataQueue[1]);
        switch (flag)
        {
        case DataTypeFlag::Acc:
            state.acc[0] = usData[0] / 32768.0f * 16.0f;
            state.acc[1] = usData[1] / 32768.0f * 16.0f;
            state.acc[2] = usData[2] / 32768.0f * 16.0f;
            newDataFlag = static_cast<DataUpdateFlag>(static_cast<unsigned char>(newDataFlag) |
                                                      static_cast<unsigned char>(DataUpdateFlag::Acc));
            break;
        case DataTypeFlag::Gyro:
            state.gyro[0] = usData[0] / 32768.0f * 2000.0f;
            state.gyro[1] = usData[1] / 32768.0f * 2000.0f;
            state.gyro[2] = usData[2] / 32768.0f * 2000.0f;
            newDataFlag = static_cast<DataUpdateFlag>(static_cast<unsigned char>(newDataFlag) |
                                                      static_cast<unsigned char>(DataUpdateFlag::Gyro));
            break;
        case DataTypeFlag::Angle:
            state.angle[0] = usData[0] / 32768.0f * 180.0f;
            state.angle[1] = usData[1] / 32768.0f * 180.0f;
            state.angle[2] = usData[2] / 32768.0f * 180.0f;
            newDataFlag = static_cast<DataUpdateFlag>(static_cast<unsigned char>(newDataFlag) |
                                                      static_cast<unsigned char>(DataUpdateFlag::Angle));
            break;
        case DataTypeFlag::Mangnetic:
        default:
            break;
        }
        dataQueue.erase(dataQueue.begin(), dataQueue.begin() + 11);
    }
}
} // namespace AD::Special
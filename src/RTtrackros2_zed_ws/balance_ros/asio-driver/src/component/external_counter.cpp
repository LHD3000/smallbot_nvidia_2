#include "asio_driver/component/external_counter.hpp"

namespace AD
{
ExternalCounter::ExternalCounter(AsioContext::SharePtr context) : pin(context)
{
    this->context = context;
}

ExternalCounter::~ExternalCounter()
{
    this->Close();
}

bool ExternalCounter::Open(int pin, GPIO::GPIOEdge edge)
{
    if (!this->pin.Open(pin))
    {
        return false;
    }
    this->pin.SetWorkMode(GPIO::GPIOMode::Interrupt);
    this->pin.SetEdge(edge);
    this->pin.OnInterrupt([this]() { this->count++; });
    return true;
}

bool ExternalCounter::Close()
{
    return this->pin.Close();
}

int ExternalCounter::Read(bool clear)
{
    int temp = count;
    if (clear)
        count = 0;
    return temp;
}
} // namespace AD
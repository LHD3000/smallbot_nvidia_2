#pragma once
#include "asio_driver/framework/asio_context.hpp"
#include "asio_driver/io/gpio.hpp"

namespace AD
{
class ExternalCounter
{
  private:
    int count;
    AsioContext::SharePtr context;
    GPIO pin;

  public:
    ExternalCounter(AsioContext::SharePtr context);
    ~ExternalCounter();
    bool Open(int pin, GPIO::GPIOEdge edge);
    bool Close();
    int Read(bool clear = true);
};
} // namespace AD

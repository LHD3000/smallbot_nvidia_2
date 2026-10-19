#pragma once

#include <functional>
#include <string>

#include "asio_driver/framework/asio_context.hpp"
namespace AD
{
class GPIO
{
  public:
    enum class GPIOEdge
    {
        Rising,
        Falling,
        Both,
        None
    };
    enum class GPIOMode
    {
        Output,
        Input,
        Interrupt,
    };

  private:
  int pinNumber;
    std::string pin;
    int fd;
    AsioContext::SharePtr context;
    std::function<void()> callback;
    bool isOpen = false;
    bool setCallback = false;

  public:
    /**
     * @param pin Pin name; e.g. "GPIO17"
     */
    GPIO(AsioContext::SharePtr context);
    ~GPIO();
    bool Open(int pinNumber);
    bool Close();
    void SetWorkMode(GPIOMode mode);
    void SetEdge(GPIOEdge edge);
    void Write(bool high);
    bool Read();
    bool IsOpen() const;
    void OnInterrupt(std::function<void()> callback);

  private:
    void HandleEvent(const epoll_event &event);
};
} // namespace AD

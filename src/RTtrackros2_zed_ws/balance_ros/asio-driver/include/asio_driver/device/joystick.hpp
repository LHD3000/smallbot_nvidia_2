#pragma once

#include "asio_driver/framework/asio_context.hpp"
#include <array>
#include <cassert>
#include <cstddef>
#include <string>

namespace AD
{
class Joystick
{
  public:
    struct JSFrame
    {
        constexpr static size_t AixCount = 8;
        constexpr static size_t ButtonCount = 17;
        std::array<float, AixCount> aixes;
        std::array<bool, ButtonCount> buttons;
        float GetAix(int code)
        {
            assert(code >= 0 && code < AixCount);
            return aixes[code];
        }
        bool GetButton(int code)
        {
            assert(code >= 0 && code < ButtonCount);
            return buttons[code];
        }
    };

  private:
    AsioContext::SharePtr context;
    int fd = -1;
    JSFrame frame;
    bool hasNew = false;
    bool isOpen = true;

  public:
    Joystick(AsioContext::SharePtr context);
    ~Joystick();
    bool Open(std::string device);
    bool Close();
    bool IsOpen();
    bool HasNewData() const;
    const JSFrame &ReadFrame();

  private:
    void OnJSEvent(const epoll_event &event);
};
} // namespace AD
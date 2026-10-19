#pragma once

#include "asio_driver/framework/asio_context.hpp"
#include <chrono>
#include <sys/timerfd.h>

namespace AD
{
class Timer
{
  private:
    AsioContext::SharePtr context;
    int timerfd;
    std::function<void()> callback;
    bool once = false;
    bool init = false;

  public:
    Timer(AsioContext::SharePtr context);
    ~Timer();
    void AsLoopAlarm(std::chrono::milliseconds interval, std::function<void()> callback);
    void AsOnceAlarm(std::chrono::milliseconds delay, std::function<void()> callback);
    bool IsInit() const;
    void Start();
    void Stop();

  private:
    void HandleEvent(const epoll_event &event);
};
} // namespace AD

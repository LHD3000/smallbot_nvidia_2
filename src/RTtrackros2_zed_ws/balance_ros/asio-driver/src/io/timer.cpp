#include "asio_driver/io/timer.hpp"

namespace AD
{
Timer::Timer(AsioContext::SharePtr context)
{
    this->context = context;

    timerfd = timerfd_create(CLOCK_REALTIME, 0);
    if (timerfd == -1)
    {
        throw std::runtime_error("Failed to create timerfd");
    }
}

Timer::~Timer()
{
    Stop();
    if (timerfd != -1)
    {
        close(timerfd);
    }
}

bool Timer::IsInit() const
{
    return init;
}

void Timer::AsLoopAlarm(std::chrono::milliseconds interval, std::function<void()> callback)
{
    once = false;
    init = true;
    this->callback = callback;

    struct itimerspec newValue;
    newValue.it_value.tv_sec = interval.count() / 1000;
    newValue.it_value.tv_nsec = interval.count() % 1000;
    newValue.it_interval.tv_sec = interval.count() / 1000;
    newValue.it_interval.tv_nsec = interval.count() % 1000;
    if (timerfd_settime(timerfd, 0, &newValue, nullptr) == -1)
    {
        throw std::runtime_error("Failed to set timer");
    }
}
void Timer::AsOnceAlarm(std::chrono::milliseconds delay, std::function<void()> callback)
{
    once = true;
    init = true;
    this->callback = callback;

    struct itimerspec newValue;
    newValue.it_value.tv_sec = delay.count() / 1000;
    newValue.it_value.tv_nsec = delay.count() % 1000;
    newValue.it_interval.tv_sec = 0;
    newValue.it_interval.tv_nsec = 0;
    if (timerfd_settime(timerfd, 0, &newValue, nullptr) == -1)
    {
        throw std::runtime_error("Failed to set timer");
    }
}

void Timer::Start()
{
    if (!init)
        throw std::runtime_error("Timer not initialized");
    context->AddHandler(timerfd, EPOLLIN | EPOLLET, std::bind(&Timer::HandleEvent, this, std::placeholders::_1));
}

void Timer::Stop()
{
    context->RemoveHandler(timerfd);
}

void Timer::HandleEvent(const epoll_event &event)
{
    uint64_t exp;
    ssize_t ret = read(timerfd, &exp, sizeof(exp));
    if (ret != sizeof(uint64_t))
    {
        throw std::runtime_error("Failed to read from timerfd");
    }

    callback();
    if (once)
    {
        Stop();
    }
}
} // namespace AD
#include "asio_driver/framework/asio_context.hpp"
#include <stdexcept>
#include <thread>

namespace AD
{
AsioContext::AsioContext()
{
    epollFD = epoll_create1(0);
    if (-1 == epollFD)
    {
        throw std::runtime_error("epoll_create1 failed");
    }
    events = std::make_unique<epoll_event[]>(MaxEvents);
}

AsioContext::~AsioContext()
{
    if (-1 != epollFD)
    {
        close(epollFD);
    }
}

AsioContext::SharePtr AsioContext::CreatAsioContext()
{
    return std::shared_ptr<AsioContext>(new AsioContext());
}

void AsioContext::Quit()
{
    running = false;
}

bool AsioContext::Running() const
{
    return running;
}

void AsioContext::SpinOnce(bool waitEvent)
{
    int nfds = epoll_wait(epollFD, events.get(), MaxEvents, waitEvent ? -1 : 0);
    if (nfds == -1)
    {
        throw std::runtime_error("epoll_wait failed");
    }
    for (int i = 0; i < nfds; i++)
    {
        handlers[events[i].data.fd](events[i]);
    }
}

void AsioContext::Spin()
{
    while (running)
    {
        SpinOnce(true);
    }
}

void AsioContext::WaitRate(int rate)
{
    std::this_thread::sleep_until(lastTime + std::chrono::microseconds(1000000 / rate));
    lastTime = std::chrono::steady_clock::now();
}

bool AsioContext::ChangeEvent(int fd, uint32_t events)
{
    struct epoll_event event;
    event.data.fd = fd;
    event.events = events;
    int ret = epoll_ctl(epollFD, EPOLL_CTL_MOD, fd, &event);
    return ret == 0;
}

bool AsioContext::AddHandler(int fd, uint32_t events, std::function<void(const epoll_event &)> handler)
{
    struct epoll_event event;
    event.data.fd = fd;
    event.events = events;
    int ret = epoll_ctl(epollFD, EPOLL_CTL_ADD, fd, &event);
    if (ret == -1)
        return false;
    handlers[fd] = handler;
    return true;
}

bool AsioContext::RemoveHandler(int fd)
{
    int ret = epoll_ctl(epollFD, EPOLL_CTL_DEL, fd, nullptr);
    if (ret == -1)
        return false;
    handlers.erase(fd);
    return true;
}

} // namespace AD
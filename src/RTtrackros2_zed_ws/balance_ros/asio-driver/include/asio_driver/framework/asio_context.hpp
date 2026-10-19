#pragma once
#include <chrono>
#include <fcntl.h>
#include <functional>
#include <map>
#include <memory>
#include <sys/epoll.h>
#include <unistd.h>
#include <vector>

namespace AD
{
class AsioContext
{
  private:
    int epollFD;
    std::unique_ptr<epoll_event[]> events;
    std::unordered_map<int, std::function<void(const epoll_event &)>> handlers;
    const int MaxEvents = 1000;
    bool running = true;
    std::chrono::steady_clock::time_point lastTime;

  public:
    ~AsioContext();
    using SharePtr = std::shared_ptr<AsioContext>;
    static SharePtr CreatAsioContext();

    void SpinOnce(bool waitEvent = false);
    void Spin();
    void WaitRate(int rate);

    bool ChangeEvent(int fd, uint32_t events);
    bool AddHandler(int fd, uint32_t events, std::function<void(const epoll_event &)> handler);
    bool RemoveHandler(int fd);

    void Quit();
    bool Running() const;

  private:
    AsioContext();
};
} // namespace AD
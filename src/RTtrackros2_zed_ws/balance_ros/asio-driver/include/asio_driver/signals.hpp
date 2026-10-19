#pragma once

#include <cassert>
#include <csignal>
namespace AD
{
enum class SignalType
{
    Interrupt = 2,  // 中断信号 ctrl-c
    Terminate = 15, // 终止信号
    Kill = 9,       // 强制终止信号
    Quit = 3,       // 退出信号
    Hangup = 1,     // 挂起信号
    User1 = 10,     // 用户自定义信号1
    User2 = 12,     // 用户自定义信号2
    Stop = 17,      // 停止信号
    Continue = 18   // 继续信号
};

inline void SignalRegeist(SignalType type, void (*handler)(int))
{
    assert(handler != nullptr);
    std::signal(static_cast<int>(type), handler);
}
inline void SignalMute(SignalType type)
{
    std::signal(SIGINT, SIG_IGN);
}
inline void SignalRemove(SignalType type)
{
    std::signal(static_cast<int>(type), SIG_DFL);
}
} // namespace AD
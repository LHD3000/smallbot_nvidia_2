# ASIO-Driver
Base on async IO operations framework epoll to support various hardware drivers in Linux.
## Introduction
This project is aimed to provide a set of drivers for various hardwares on linux.
At same time, it also provides a framework for async IO operations based on epoll.
## Features
+ Async

  So that the drivers can handle multiple requests at the same time.
+ Event driven

  Based on epoll, the drivers can handle events in a non-blocking way only use a small amount of CPU.
+ Easy to use

  The drivers are easy to use and can be easily integrated into other projects.
+ Easy to extend

  The drivers can be easily extended to support new hardware or new protocols.

## Usage Example
As you can see in the example below, we can use ASIO-Driver to send and receive messages only with a few lines of code.
```c++
int main()
{
    // create asio context
    auto context = AD::AsioContext::CreatAsioContext();

    //create serial io instance
    AD::Serial serial(context);

    //open serial port
    serial.Open("/dev/ttyUSB0");

    //regeist listener for receive event
    serial.OnReceive([&](const std::span<const char> &data)
                     {
                        std::cout << std::string(data.begin(), data.end()) << std::endl;
                     });

    //send message
    std::string message = "Hello, world!";
    serial.Send(std::span<const char>(message.begin(), message.size()));

    //enter event loop
    context->Spin();
    return 0;
}
```

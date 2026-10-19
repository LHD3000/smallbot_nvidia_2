#include "asio_driver/io/gpio.hpp"
#include <cassert>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace AD
{

static void ExportGPIO(int pin)
{
    std::string exportPath = "/sys/class/gpio/export";
    FILE *fp = fopen(exportPath.c_str(), "w");
    if (fp == nullptr)
        throw std::runtime_error("Failed to open export file, with error: " + std::string(strerror(errno)));
    fprintf(fp, "%s", std::to_string(pin).c_str());
    fclose(fp);
}

static void UnexportGPIO(int pin)
{
    std::string unexportPath = "/sys/class/gpio/unexport";
    FILE *fp = fopen(unexportPath.c_str(), "w");
    if (fp == nullptr)
        throw std::runtime_error("Failed to open unexport file");
    fprintf(fp, "%s", std::to_string(pin).c_str());
    fclose(fp);
}

GPIO::GPIO(AsioContext::SharePtr context)
{
    this->context = context;
}

GPIO::~GPIO()
{
    if (isOpen)
        Close();
}

bool GPIO::Open(int pinNumber)
{
    this->pinNumber = pinNumber;
    this->pin = "gpio" + std::to_string(pinNumber);
    std::string pinPath = "/sys/class/gpio/" + pin;
    if (0 != access(pinPath.c_str(), F_OK))
    {
        ExportGPIO(pinNumber);
        usleep(10000);
        std::cout << "Exported GPIO pin: " << pin << std::endl;
    }
    std::string valuePath = "/sys/class/gpio/" + pin + "/value";
    fd = open(valuePath.c_str(), O_RDWR);
    if (fd < 0)
        throw std::runtime_error("Failed to open GPIO pin" + std::string(strerror(errno)));
    isOpen = true;
    return isOpen;
}

bool GPIO::Close()
{
    if (setCallback)
        context->RemoveHandler(fd);
    if (fd > 0)
    {
        close(fd);
        fd = -1;
    }
    std::string pinPath = "/sys/class/gpio/" + pin;
    if (0 != access(pinPath.c_str(), F_OK))
    {
        UnexportGPIO(pinNumber);
        std::cout << "Unexported GPIO pin: " << pin << std::endl;
        return true;
    }
    return false;
}

bool GPIO::IsOpen() const
{
    return isOpen;
}

void GPIO::SetWorkMode(GPIO::GPIOMode mode)
{
    std::string directionPath = "/sys/class/gpio/" + pin + "/direction";
    FILE *dirfp = fopen(directionPath.c_str(), "w");
    switch (mode)
    {
    case GPIO::GPIOMode::Output:
        fprintf(dirfp, "out");
        break;
    case GPIO::GPIOMode::Input:
        fprintf(dirfp, "in");
        break;
    case GPIO::GPIOMode::Interrupt:
        fprintf(dirfp, "in");
        context->AddHandler(fd, EPOLLPRI, std::bind(&GPIO::HandleEvent, this, std::placeholders::_1));
        break;
    }
    fclose(dirfp);
}

void GPIO::SetEdge(GPIO::GPIOEdge edge)
{
    std::string edgePath = "/sys/class/gpio/" + pin + "/edge";
    FILE *fp = fopen(edgePath.c_str(), "w");
    if (edge == GPIOEdge::Rising)
    {
        fprintf(fp, "rising");
    }
    else if (edge == GPIOEdge::Falling)
    {
        fprintf(fp, "falling");
    }
    else if (edge == GPIOEdge::Both)
    {
        fprintf(fp, "both");
    }
    else if (edge == GPIOEdge::None)
    {
        fprintf(fp, "none");
    }
    fclose(fp);
}

void GPIO::Write(bool high)
{
    std::string valuePath = "/sys/class/gpio/" + pin + "/value";
    FILE *fp = fopen(valuePath.c_str(), "w");
    if (high)
    {
        fprintf(fp, "1");
    }
    else
    {
        fprintf(fp, "0");
    }
    fclose(fp);
}

bool GPIO::Read()
{
    std::string valuePath = "/sys/class/gpio/" + pin + "/value";
    FILE *fp = fopen(valuePath.c_str(), "r");
    char value;
    fread(&value, 1, 1, fp);
    fclose(fp);
    return '1' == value;
}

void GPIO::OnInterrupt(std::function<void()> callback)
{
    assert(callback != nullptr);
    this->callback = callback;
    if (setCallback)
        context->RemoveHandler(fd);
    setCallback = context->AddHandler(fd, EPOLLPRI, std::bind(&GPIO::HandleEvent, this, std::placeholders::_1));
}

void GPIO::HandleEvent(const epoll_event &event)
{
    char value;
    read(fd, &value, 1);
    if (event.events & EPOLLPRI)
    {
        assert(callback != nullptr);
        callback();
    }
}
} // namespace AD
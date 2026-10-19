#include "asio_driver/device/joystick.hpp"
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/epoll.h>

namespace AD
{
Joystick::Joystick(AsioContext::SharePtr context) : context(context) {}
Joystick::~Joystick()
{
    Close();
}
bool Joystick::Open(std::string device)
{
    if (isOpen)
    {
        Close();
    }
    fd = open(device.c_str(), O_RDWR);
    if (fd < 0)
    {
        isOpen = false;
        return isOpen;
    }

    isOpen = context->AddHandler(fd, EPOLLIN | EPOLLET, std::bind(&Joystick::OnJSEvent, this, std::placeholders::_1));
    return isOpen;
}

bool Joystick::Close()
{
    if (isOpen && (-1 == close(fd) || !context->RemoveHandler(fd)))
    {
        isOpen = false;
        return false;
    }
    return true;
}

bool Joystick::IsOpen()
{
    return isOpen;
}

bool Joystick::HasNewData() const
{
    return hasNew;
}
const Joystick::JSFrame &Joystick::ReadFrame()
{
    hasNew = false;
    return frame;
}

enum JSEventType : unsigned char
{
    Button = 0x01,
    Axis = 0x02,
    Init = 0x80
};

void Joystick::OnJSEvent(const epoll_event &event)
{
    if (event.events & EPOLLIN)
    {
        js_event jsEvent;
        // TODO if only read once, some event will be lost. unluckly we can never read all event, it alawys return 8
        // bytes.
        for (int i = 0; i < 4; i++)
        {
            ssize_t ret = read(fd, &jsEvent, sizeof(jsEvent));
            if (ret <= 0)
            {
                return;
            }
            jsEvent.type &= ~JSEventType::Init;
            if (jsEvent.type == JSEventType::Axis)
            {
                frame.aixes[jsEvent.number] = jsEvent.value / 32767.0f;
                hasNew = true;
            }
            else if (jsEvent.type == JSEventType::Button)
            {
                frame.buttons[jsEvent.number] = jsEvent.value == 1;
                hasNew = true;
            }
        }
    }
}
} // namespace AD
#include "asio_driver/device/buzzer_src.hpp"
#include <iostream>
#include <thread>

#include "../test_config.hpp"

int main()
{
    std::cout << "TEST:buzzer" << std::endl;

    auto context = AD::AsioContext::CreatAsioContext();
    AD::Buzzer buzzer = AD::Buzzer(context);
    buzzer.Open(BuzzerPin, BuzzerActiveHigh);
    buzzer.Beep();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    buzzer.StopBeep();
    buzzer.Close();
    std::cout << "TEST FINISHED" << std::endl;
}
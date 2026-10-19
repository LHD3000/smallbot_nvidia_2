#include "asio_driver/component/external_counter.hpp"
#include "asio_driver/framework/rate_controller.hpp"
#include <iostream>

int main()
{
    std::cout << "TEST:external_counter" << std::endl;

    AD::AsioContext::SharePtr context = AD::AsioContext::CreatAsioContext();
    AD::ExternalCounter counter(context);

    counter.Open(12, AD::GPIO::GPIOEdge::Rising);
    AD::RateController rate_200hz(200);
    while (context->Running())
    {
        if (rate_200hz.IsArrive())
        {
            std::cout << "Counter: " << counter.Read() << std::endl;
        }
        context->SpinOnce();
    }
    counter.Close();

    return 0;
}
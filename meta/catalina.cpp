// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "gpio.hpp"
#include "meta.hpp"

#include <chrono>
#include <iostream>

namespace meta
{

sdbusplus::async::task<bool> catalina_cmos_reset(sdbusplus::async::context& ctx)
{
    int gpiochipint =
        gpio::find_chip_idx_from_dir("/sys/bus/i2c/devices/6-0021");

    if (gpiochipint < 0)
    {
        std::cerr << "Error: couldn't find gpiochip\n";
        co_return false;
    }

    const char* rtcClr = "RTC_CLR_L";
    const size_t TIMEOUT = 5;

    gpio::set(rtcClr, 0);
    co_await sdbusplus::async::sleep_for(ctx, std::chrono::seconds(TIMEOUT));
    gpio::set(rtcClr, 1);
    co_return true;
}

int init_catalina_base()
{
    std::cerr << "Catalina platform init \n";
    return EXIT_SUCCESS;
}
} // namespace meta

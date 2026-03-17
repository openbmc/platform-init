// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "gpio.hpp"
#include "meta.hpp"

#include <iostream>
#include <thread>

namespace meta
{

void catalina_cmos_reset()
{
    std::cerr << "CMOS reset triggered\n";

    const char* rtcClr = "RTC_CLR_L";

    gpio::set(rtcClr, 0);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    gpio::set(rtcClr, 1);

    return;
}

int init_catalina_base()
{
    std::cerr << "Catalina platform init \n";

    return EXIT_SUCCESS;
}
} // namespace meta

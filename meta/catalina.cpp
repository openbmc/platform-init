// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "meta.hpp"

#include <chrono>
#include <iostream>

namespace meta
{

sdbusplus::async::task<bool> catalina_cmos_reset(sdbusplus::async::context& ctx)
{
    std::cerr << "CMOS reset triggered\n";
    sdbusplus::async::sleep_for(ctx, std::chrono::seconds(5));
    co_return true;
}

int init_catalina_base()
{
    std::cerr << "Catalina platform init \n";

    return EXIT_SUCCESS;
}
} // namespace meta

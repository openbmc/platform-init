// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "meta.hpp"

#include <systemd/sd-daemon.h>

#include <chrono>

namespace meta
{

sdbusplus::async::task<bool> catalina_cmos_reset(sdbusplus::async::context& ctx)
{
    info("CMOS reset triggered\n");
    sdbusplus::async::sleep_for(ctx, std::chrono::seconds(5));
    co_return true;
}

int init_catalina_base()
{
    info("Catalina platform init");

    int notify_rc = sd_notify(0, "READY=1\nSTATUS=Catalina init complete");

    if (notify_rc < 0)
    {
        error("sd_notify READY=1 failed: {RC}", "RC", notify_rc);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
} // namespace meta

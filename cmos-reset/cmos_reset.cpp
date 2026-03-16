// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "cmos_reset.hpp"

namespace cmos
{

sdbusplus::async::task<> CmosReset::startReset()
{
    info("CMOS reset requested");

    bool success = co_await doReset();

    if (!success)
    {
        error("CMOS reset failed");
        co_return;
    }

    info("CMOS reset successful");
}

sdbusplus::async::task<bool> CmosReset::doReset()
{
    if (!resetMethod)
    {
        error("No reset function provided for this platform");
        co_return false;
    }

    bool success = co_await resetMethod(ctx);
    if (!success)
    {
        error("CMOS reset failed");
        co_return false;
    }

    co_return true;
}

int init_service(CmosResetMethod resetMethod)
{
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager(ctx, softwarePath);

    CmosReset service(ctx, softwarePath, resetMethod);

    ctx.spawn([](sdbusplus::async::context& ctx) -> sdbusplus::async::task<> {
        ctx.request_name(busName);
        co_return;
    }(ctx));

    ctx.run();

    return EXIT_SUCCESS;
}
} // namespace cmos

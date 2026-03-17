// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "cmos_reset.hpp"

#include "host_power.hpp"

#include <phosphor-logging/commit.hpp>
#include <phosphor-logging/lg2.hpp>

PHOSPHOR_LOG2_USING;

namespace cmos
{

using namespace host_power;

sdbusplus::async::task<> CmosReset::startReset()
{
    info("CMOS reset requested");

    bool success = co_await doReset();

    if (!success)
    {
        error("CMOS reset failed");
        co_await lg2::commit(
            ctx, ResetFailure("RESET_CAUSE", "Manual CMOS reset", "SOURCE", 
                              sdbusplus::object_path(softwarePath)));
        co_return;
    }

    info("CMOS reset successful");
    const auto logPath = co_await lg2::commit(
        ctx, ResetSuccess("RESET_CAUSE", "Manual CMOS reset", "SOURCE",
                          sdbusplus::object_path(softwarePath)));

    co_await lg2::resolve(ctx, logPath);
}

sdbusplus::async::task<bool> CmosReset::doReset()
{
    if (!resetMethod)
    {
        error("No reset function provided for this platform");
        co_return false;
    }

    // NOLINTNEXTLINE(clang-analyzer-core.uninitialized.Branch)
    auto powerstate = co_await HostPower::getState(ctx);

    if (powerstate != stateOn && powerstate != stateOff)
    {
        error("Invalid host state {STATE}", "STATE", powerstate);
        co_return false;
    }

    bool restorePowerState = false;
    if (powerstate == stateOn)
    {
        info("Host is running, transitioning to off");
        bool success = co_await HostPower::setState(ctx, stateOff);

        if (!success)
        {
            error("Error host power off");
            co_return false;
        }

        restorePowerState = true;
    }

    bool success = co_await resetMethod(ctx);
    if (!success)
    {
        error("CMOS reset failed");
        co_return false;
    }

    if (restorePowerState)
    {
        lg2::info("Restoring previous host state");
        bool success = co_await HostPower::setState(ctx, stateOn);

        if (!success)
        {
            error("Error restoring power state");
            co_return false;
        }
        lg2::info("Host state restored successfully");
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

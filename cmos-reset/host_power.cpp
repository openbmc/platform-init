#include "host_power.hpp"

namespace host_power
{

sdbusplus::async::task<bool> HostPower::setState(sdbusplus::async::context& ctx,
                                                 HostState state)
{
    auto client = sdbusplus::client::xyz::openbmc_project::state::Host(ctx)
                      .service(hostStateService)
                      .path(host0ObjectPath);

    co_await client.requested_host_transition(
        (state == stateOn) ? transitionOn : transitionOff);

    debug("Requested host transition to {STATE}", "STATE", state);

    constexpr size_t transitionTimeout = HOST_STATE_TRANSITION_TIMEOUT;

    for (size_t i = 0; i < transitionTimeout; i++)
    {
        co_await sdbusplus::async::sleep_for(ctx, std::chrono::seconds(1));

        if ((co_await client.current_host_state()) == state)
        {
            debug("Successfully achieved state {STATE}", "STATE", state);
            co_return true;
        }
    }

    error("Failed to achieve state {STATE} before the timeout of {TIMEOUT}s",
          "STATE", state, "TIMEOUT", transitionTimeout);

    co_return false;
}

sdbusplus::async::task<HostState> HostPower::getState(
    sdbusplus::async::context& ctx)
{
    auto client = sdbusplus::client::xyz::openbmc_project::state::Host(ctx)
                      .service(hostStateService)
                      .path(host0ObjectPath);

    auto res = co_await client.current_host_state();

    co_return res;
}
} // namespace host_power

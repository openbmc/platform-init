#pragma once

#include "config.h"

#include <sdbusplus/async.hpp>
#include <sdbusplus/async/context.hpp>
#include <xyz/openbmc_project/State/Host/client.hpp>

namespace host_power
{

const auto stateOn =
    sdbusplus::client::xyz::openbmc_project::state::Host<>::HostState::Running;
const auto stateOff =
    sdbusplus::client::xyz::openbmc_project::state::Host<>::HostState::Off;
const auto transitionOn =
    sdbusplus::client::xyz::openbmc_project::state::Host<>::Transition::On;
const auto transitionOff =
    sdbusplus::client::xyz::openbmc_project::state::Host<>::Transition::Off;

using HostState =
    sdbusplus::client::xyz::openbmc_project::state::Host<>::HostState;

using StateIntf =
    sdbusplus::client::xyz::openbmc_project::state::Host<void, void>;

const auto host0ObjectPath = sdbusplus::client::xyz::openbmc_project::state::
                                 Host<>::namespace_path::value +
                             std::string("/host0");

constexpr const char* hostStateService = "xyz.openbmc_project.State.Host0";

class HostPower
{
  public:
    // @param state   desired powerstate
    // @returns       true on success
    static sdbusplus::async::task<bool> setState(sdbusplus::async::context& ctx,
                                                 HostState state);

    // @returns       host powerstate
    static sdbusplus::async::task<HostState> getState(
        sdbusplus::async::context& ctx);
};
}; // namespace host_power

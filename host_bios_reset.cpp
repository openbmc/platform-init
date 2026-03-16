// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "host_bios_reset.hpp"

#include <sdbusplus/bus.hpp>

namespace host
{
namespace bios
{
namespace reset
{

void BiosReset::reset()
{
    if (resetMethod)
    {
        resetMethod();
    }
    return;
}

void init_service(BiosResetMethod resetMethod)
{
    sdbusplus::bus_t bus = sdbusplus::bus::new_default();

    sdbusplus::server::manager_t objManager(bus,
                                            host::bios::reset::softwarePath);

    sdbusplus::message::object_path instancePath(
        host::bios::reset::softwarePath);
    instancePath /= std::format("host{}", 0);
    instancePath /= "bios";

    BiosReset service{bus, instancePath.str.c_str(), resetMethod};

    bus.request_name(host::bios::reset::busName);

    bus.process_loop();
}
} // namespace reset
} // namespace bios
} // namespace host

// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "cmos_reset.hpp"

#include <sdbusplus/bus.hpp>

namespace cmos
{
namespace reset
{

void CmosReset::reset()
{
    if (resetMethod)
    {
        resetMethod();
    }
    return;
}

void init_service(CmosResetMethod resetMethod)
{
    sdbusplus::bus_t bus = sdbusplus::bus::new_default();

    sdbusplus::server::manager_t objManager(bus, softwarePath);

    CmosReset service{bus, softwarePath, resetMethod};

    bus.request_name(cmos::reset::busName);

    bus.process_loop();
}
} // namespace reset
} // namespace cmos

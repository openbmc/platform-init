// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "gpio.hpp"

#include <systemd/sd-daemon.h>

#include <iostream>

namespace intel
{

int init_jcrp()
{
    std::cerr << "Initializing Intel JCRP platform...\n";

    // skipping FRU validation and S2D discovery as it's not relevant at this
    // stage
    gpio::set("HPM_FRU_READ_DONE", 1);
    gpio::set("HPM0_FRU_VALID", 1);
    gpio::set("FM_S2D_DISCOVERY_DONE", 1);

    gpio::set("BMC_BOOT_DONE", 1);

    std::cerr << "Intel JCRP platform initialization complete\n";
    sd_notify(0, "READY=1");

    return EXIT_SUCCESS;
}

} // namespace intel

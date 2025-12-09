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

    gpio::set_raw(2, 19, 1);
    gpio::set_raw(2, 49, 1);
    gpio::set_raw(2, 57, 1);
    gpio::set_raw(2, 23, 1);

    std::cerr << "Intel JCRP platform initialization complete\n";
    sd_notify(0, "READY=1");

    return EXIT_SUCCESS;
}

} // namespace intel

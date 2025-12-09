// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "gpio.hpp"
#include <systemd/sd-daemon.h>
#include <iostream>

namespace intel
{

int init_acrp()
{
    std::cerr << "Initializing Intel ACRP platform...\n";
    
    gpio::set_raw(2, 199, 1);

    std::cerr << "Intel ACRP platform initialization complete\n";
    sd_notify(0, "READY=1");
    
    return EXIT_SUCCESS;
}

} // namespace intel

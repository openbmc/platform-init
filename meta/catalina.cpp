// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "host_bios_reset.hpp"
#include "meta.hpp"

#include <iostream>

namespace meta
{

void catalina_cmos_reset()
{
    std::cerr << "CMOS reset triggered\n";
}

int init_catalina_base()
{
    std::cerr << "Catalina platform init \n";

    host::bios::reset::init_service(catalina_cmos_reset);

    return EXIT_SUCCESS;
}

} // namespace meta

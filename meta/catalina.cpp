// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "meta.hpp"

#include <iostream>

namespace meta
{

int init_catalina_base()
{
    std::cerr << "Catalina platform init \n";
    return EXIT_SUCCESS;
}
} // namespace meta

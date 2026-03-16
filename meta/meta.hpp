// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include <sdbusplus/async.hpp>

namespace meta
{

// list your platform initialization callbacks here
int init_catalina_base();
sdbusplus::async::task<bool> catalina_cmos_reset(sdbusplus::async::context&);

} // namespace meta

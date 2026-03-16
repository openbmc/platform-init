// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

namespace meta
{

// list your platform initialization callbacks here
int init_catalina_base();
void catalina_cmos_reset();

} // namespace meta

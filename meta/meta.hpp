// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include <phosphor-logging/lg2.hpp>

PHOSPHOR_LOG2_USING;

namespace meta
{

// list your platform initialization callbacks here
int init_catalina_base();

} // namespace meta

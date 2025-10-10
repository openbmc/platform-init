// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

namespace nvidia
{

// list your platform initialization callbacks here
int init_gb200_base();
int init_gb200_with_p2020();
int init_nvl32();

} // namespace nvidia

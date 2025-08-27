// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2025 NVIDIA

#pragma once

#include <string>

namespace i2c {

void rebind_controller(const std::string_view number);
void new_device(unsigned int bus, unsigned int address,
		std::string_view device_type);
}

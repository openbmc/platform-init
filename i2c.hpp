// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2025 NVIDIA

#pragma once

#include <cstdint>
#include <string>

namespace i2c
{

void rebind_controller(const std::string_view number);
void new_device(unsigned int bus, unsigned int address,
                std::string_view device_type);

// a simple RAII wrapper for raw i2c comms
struct RawDevice
{
    RawDevice(size_t bus, uint8_t address);
    ~RawDevice();
    RawDevice(const RawDevice&) = delete;
    RawDevice& operator=(const RawDevice&) = delete;
    RawDevice& operator=(RawDevice&&) = default;
    RawDevice(RawDevice&&) = default;

    int read_byte(uint8_t reg, uint8_t& val);

    int fd;
};
} // namespace i2c

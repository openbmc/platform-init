// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <system_error>
#include <vector>

namespace i2c
{

void rebind_controller(const std::string_view number);
void new_device(unsigned int bus, unsigned int address,
                std::string_view device_type);

std::expected<void, std::error_code> bind_device(
    unsigned int bus, unsigned int address, std::string_view driver_name);

// a simple RAII wrapper for raw i2c comms
struct RawDevice
{
    RawDevice(size_t bus, uint8_t address);
    ~RawDevice();
    RawDevice(const RawDevice&) = delete;
    RawDevice& operator=(const RawDevice&) = delete;
    RawDevice& operator=(RawDevice&&) = default;
    RawDevice(RawDevice&&) = default;

    std::expected<uint8_t, std::error_code> read_byte(uint8_t reg);

    std::expected<void, std::error_code> write_block(
        std::span<const uint8_t> data);

    std::expected<std::vector<uint8_t>, std::error_code> read_i2c_block(
        uint8_t reg, size_t len);

    int fd;
};
} // namespace i2c

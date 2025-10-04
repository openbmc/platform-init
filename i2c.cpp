// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2025 NVIDIA

#include "i2c.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>

extern "C"
{
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

namespace i2c
{

void rebind_controller(std::string_view number)
{
    std::string bindpath =
        std::format("/sys/bus/platform/drivers/aspeed-i2c-bus/unbind", number);
    std::ofstream bindofs(bindpath);
    if (!bindofs)
    {
        std::cerr << std::format("{} unable to open\n", bindpath);
        return;
    }
    try
    {
        bindofs << std::format("{}.i2c\n", number);
    }
    catch (const std::system_error& e)
    {
        std::cerr << std::format("{} unable to write\n", bindpath);
        return;
    }
    bindofs.close();
    std::cerr << std::format("{} unbound\n", number);

    std::string unbindpath =
        std::format("/sys/bus/platform/drivers/aspeed-i2c-bus/bind", number);
    std::ofstream unbindofs(unbindpath);
    if (!unbindofs)
    {
        std::cerr << std::format("{} unable to open\n", unbindpath);
        return;
    }
    try
    {
        unbindofs << std::format("{}.i2c\n", number);
    }
    catch (const std::system_error& e)
    {
        std::cerr << std::format("{} unable to write\n", unbindpath);
        return;
    }
    std::cerr << std::format("{} bound\n", number);
}

void new_device(unsigned int bus, unsigned int address,
                std::string_view device_type)
{
    std::string path =
        std::format("/sys/bus/i2c/devices/i2c-{}/new_device", bus);
    std::cerr << std::format("attempting to open {}", path);
    std::ofstream new_device(path);
    if (!new_device)
    {
        std::cerr << "Error: Unable to create I2C device\n";
        return;
    }
    new_device << std::format("{} 0x{:02x}", device_type, address);
    new_device.close();

    std::cerr << std::format("{} device created at bus {}", device_type, bus);
}

RawDevice::RawDevice(size_t bus, uint8_t address)
{
    std::string bus_path = std::format("/dev/i2c-{}", bus);
    std::filesystem::path dev_path = bus_path;
    fd = open(dev_path.c_str(), O_RDWR);
    if (fd < 0)
    {
        std::cerr << std::format("failed to open {}\n", dev_path.native());
        throw std::runtime_error(
            std::format("Failed to open {}", dev_path.native()));
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0)
    {
        // dtor won't be called since we never finished constructing it, clean
        // up our fd
        close(fd);
        throw std::runtime_error(
            std::format("Failed to specify address {}", address));
    }
}

RawDevice::~RawDevice()
{
    close(fd);
}

std::expected<uint8_t, std::error_code> RawDevice::read_byte(uint8_t reg)
{
    int result = i2c_smbus_read_byte_data(fd, reg);
    if (result < 0)
    {
        return std::unexpected(
            std::error_code(-result, std::system_category()));
    }

    return result;
}

} // namespace i2c

// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2025 NVIDIA

#include "i2c.hpp"

#include <iostream>
#include <format>
#include <fstream>
namespace i2c {

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
}

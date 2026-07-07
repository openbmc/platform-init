// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

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

void bind_device(unsigned int bus, unsigned int address,
                 std::string_view driver_name)
{
    std::string path = std::format("/sys/bus/i2c/drivers/{}/bind", driver_name);
    std::ofstream bind_f(path);
    if (!bind_f)
    {
        std::cerr << std::format("Cannot open {}\n", path);
        return;
    }
    std::string device = std::format("{}-{:04x}", bus, address);
    bind_f << device;
    if (bind_f.fail())
    {
        std::cerr << std::format(
            "bind_device: write to {} for {} did not succeed\n", path, device);
        return;
    }
    std::cerr << std::format("bound {} to {} driver\n", device, driver_name);
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

std::expected<void, std::error_code> RawDevice::write_block(
    std::span<const uint8_t> data)
{
    if (data.empty())
    {
        return std::unexpected(
            std::make_error_code(std::errc::invalid_argument));
    }
    ssize_t n = ::write(fd, data.data(), data.size());
    if (n < 0)
    {
        return std::unexpected(std::error_code(errno, std::system_category()));
    }
    if (static_cast<size_t>(n) != data.size())
    {
        return std::unexpected(std::make_error_code(std::errc::io_error));
    }
    return {};
}

std::expected<std::vector<uint8_t>, std::error_code> RawDevice::read_i2c_block(
    uint8_t reg, size_t len)
{
    if (len == 0 || len > I2C_SMBUS_BLOCK_MAX)
    {
        return std::unexpected(
            std::make_error_code(std::errc::invalid_argument));
    }
    std::vector<uint8_t> buf(len);
    int32_t n = i2c_smbus_read_i2c_block_data(
        fd, reg, static_cast<uint8_t>(len), buf.data());
    if (n < 0)
    {
        return std::unexpected(std::error_code(-n, std::system_category()));
    }
    if (static_cast<size_t>(n) != len)
    {
        return std::unexpected(std::make_error_code(std::errc::io_error));
    }
    return buf;
}

} // namespace i2c

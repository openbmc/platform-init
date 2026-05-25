// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "ftdi_mdio.hpp"

#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace ftdi_mdio
{

namespace
{

namespace mpsse
{

namespace command
{

constexpr uint8_t clockDataBytesOutMsb = 0x10;
constexpr uint8_t clockDataBytesInMsb = 0x34;
constexpr uint8_t sendImmediate = 0x87;
constexpr uint8_t setClockDivisor = 0x86;
constexpr uint8_t disableClockDivideBy5 = 0x8a;
constexpr uint8_t disableAdaptiveClocking = 0x97;
constexpr uint8_t disableThreePhaseClocking = 0x8d;
constexpr uint8_t setAdbusGpio = 0x80;
constexpr uint8_t setAcbusGpio = 0x82;

} // namespace command

} // namespace mpsse

namespace clause22
{

constexpr uint32_t startBitShift = 30;
constexpr uint32_t startBitMask = 0x3;
constexpr uint32_t start = 0x1;

constexpr uint32_t operationBitShift = 28;
constexpr uint32_t operationBitMask = 0x3;
constexpr uint32_t operationWrite = 0x1;
constexpr uint32_t operationRead = 0x2;

constexpr uint32_t phyAddressBitShift = 23;
constexpr uint32_t phyAddressBitMask = 0x1f;

constexpr uint32_t registerAddressBitShift = 18;
constexpr uint32_t registerAddressBitMask = 0x1f;

constexpr uint32_t turnaroundBitShift = 16;
constexpr uint32_t turnaroundBitMask = 0x3;
constexpr uint32_t turnaroundRead = 0x0;
constexpr uint32_t turnaroundWrite = 0x2;

} // namespace clause22

int readSysfsInt(const std::filesystem::path& path)
{
    std::ifstream input(path);
    int value = 0;
    if (!(input >> value))
    {
        return -1;
    }

    return value;
}

int openUsbDeviceByPath(struct ftdi_context& context,
                        const std::filesystem::path& devicePath)
{
    int busNumber = readSysfsInt(devicePath / "busnum");
    int deviceNumber = readSysfsInt(devicePath / "devnum");
    if (busNumber < 0 || deviceNumber < 0)
    {
        return -ENODEV;
    }

    int openResult =
        ftdi_usb_open_bus_addr(&context, static_cast<uint8_t>(busNumber),
                               static_cast<uint8_t>(deviceNumber));
    if (openResult < 0 && openResult != -5)
    {
        return (openResult == -3) ? -ENODEV : -EIO;
    }

    return 0;
}

bool hasPathComponent(const std::filesystem::path& path,
                      std::string_view component)
{
    for (const auto& part : path)
    {
        if (part.string() == component)
        {
            return true;
        }
    }

    return false;
}

std::optional<int> parseUsbBusNumber(const std::string& usbRootName)
{
    constexpr std::string_view usbPrefix = "usb";
    if (!usbRootName.starts_with(usbPrefix))
    {
        return std::nullopt;
    }

    try
    {
        return std::stoi(usbRootName.substr(usbPrefix.size()));
    }
    catch (const std::exception&)
    {
        return std::nullopt;
    }
}

std::string makeUsbControllerName(std::string_view controllerAddress)
{
    constexpr std::string_view usbSuffix = ".usb";
    std::string controller(controllerAddress);
    if (!controller.ends_with(usbSuffix))
    {
        controller += usbSuffix;
    }

    return controller;
}

std::optional<std::filesystem::path> resolveUsbDevicePath(
    std::string_view controllerAddress, std::string_view location)
{
    const std::string controller = makeUsbControllerName(controllerAddress);

    const std::filesystem::path usbDevicesPath("/sys/bus/usb/devices");
    std::error_code errorCode;
    for (const auto& entry :
         std::filesystem::directory_iterator(usbDevicesPath, errorCode))
    {
        const std::string usbRootName = entry.path().filename().string();
        const std::optional<int> busNumber = parseUsbBusNumber(usbRootName);
        if (!busNumber)
        {
            continue;
        }

        const std::filesystem::path usbRootPath =
            std::filesystem::canonical(entry.path(), errorCode);
        if (errorCode)
        {
            errorCode.clear();
            continue;
        }

        if (!hasPathComponent(usbRootPath, controller))
        {
            continue;
        }

        const std::filesystem::path devicePath =
            usbDevicesPath /
            (std::to_string(*busNumber) + "-" + std::string(location));
        if (std::filesystem::exists(devicePath, errorCode))
        {
            return devicePath;
        }
        errorCode.clear();
    }

    return std::nullopt;
}

uint32_t makeClause22Frame(uint8_t operation, uint8_t phyAddress,
                           uint8_t registerAddress, uint8_t turnaround)
{
    uint32_t frame = 0;
    frame |= (clause22::start & clause22::startBitMask)
             << clause22::startBitShift;
    frame |= (operation & clause22::operationBitMask)
             << clause22::operationBitShift;
    frame |= (phyAddress & clause22::phyAddressBitMask)
             << clause22::phyAddressBitShift;
    frame |= (registerAddress & clause22::registerAddressBitMask)
             << clause22::registerAddressBitShift;
    frame |= (turnaround & clause22::turnaroundBitMask)
             << clause22::turnaroundBitShift;
    return frame;
}

class MpssePacketBuilder
{
  public:
    void beginClockData(uint8_t command)
    {
        append(command);
        lengthPosition = position;
        append(0x00);
        append(0x00);
        payloadStart = position;
    }

    void endClockData()
    {
        const std::size_t payloadLength = position - payloadStart;
        const std::size_t encodedLength = payloadLength - 1;
        packet[lengthPosition] = static_cast<uint8_t>(encodedLength & 0xff);
        packet[lengthPosition + 1] =
            static_cast<uint8_t>((encodedLength >> 8) & 0xff);
        clockDataLength = payloadLength;
    }

    void append(uint8_t value)
    {
        packet[position++] = value;
    }

    void appendIdle()
    {
        append(0xff);
    }

    void appendBigEndian16(uint16_t value)
    {
        append(static_cast<uint8_t>((value >> 8) & 0xff));
        append(static_cast<uint8_t>(value & 0xff));
    }

    void appendClause22Header(uint32_t frame)
    {
        appendIdle();
        appendIdle();
        appendIdle();
        appendIdle();
        append(static_cast<uint8_t>((frame >> 24) & 0xff));
        append(static_cast<uint8_t>((frame >> 16) & 0xff));
    }

    std::size_t clockDataPayloadSize() const
    {
        if (clockDataLength != 0)
        {
            return clockDataLength;
        }

        return position - payloadStart;
    }

    const uint8_t* data() const
    {
        return packet.data();
    }

    int size() const
    {
        return static_cast<int>(position);
    }

  private:
    std::array<uint8_t, 12> packet = {};
    std::size_t position = 0;
    std::size_t lengthPosition = 0;
    std::size_t payloadStart = 0;
    std::size_t clockDataLength = 0;
};

} // namespace

Mdio::Mdio(const DeviceConfig& config)
{
    if (ftdi_init(&context) < 0)
    {
        throw std::runtime_error("failed to initialize ftdi context");
    }
    initialized = true;

    if (openDevice(config) < 0)
    {
        if (opened)
        {
            ftdi_usb_close(&context);
            opened = false;
        }
        ftdi_deinit(&context);
        initialized = false;
        throw std::runtime_error("failed to open ftdi mdio device");
    }
}

Mdio::~Mdio()
{
    if (opened)
    {
        ftdi_usb_close(&context);
    }

    if (initialized)
    {
        ftdi_deinit(&context);
    }
}

int Mdio::openDevice(const DeviceConfig& config)
{
    if (ftdi_set_interface(&context, config.interface) < 0)
    {
        return -EIO;
    }

    const std::optional<std::filesystem::path> devicePath =
        resolveUsbDevicePath(config.controllerAddress, config.location);
    if (!devicePath)
    {
        return -ENODEV;
    }

    int openResult = openUsbDeviceByPath(context, *devicePath);
    if (openResult < 0)
    {
        return openResult;
    }
    opened = true;

    return initializeOpenedDevice();
}

int Mdio::initializeOpenedDevice()
{
    if (configurePort() < 0)
    {
        return -EIO;
    }

    const uint8_t mpsseSetup[] = {
        mpsse::command::disableClockDivideBy5,
        mpsse::command::disableAdaptiveClocking,
        mpsse::command::disableThreePhaseClocking,
    };
    if (writeData(mpsseSetup, sizeof(mpsseSetup)) < 0)
    {
        return -EIO;
    }

    if (setClockDivider(59) < 0)
    {
        return -EIO;
    }

    const uint8_t adbusGpioSetup[] = {
        mpsse::command::setAdbusGpio,
        0x01, // value
        0x03, // direction
    };
    if (writeData(adbusGpioSetup, sizeof(adbusGpioSetup)) < 0)
    {
        return -EIO;
    }

    const uint8_t acbusGpioSetup[] = {
        mpsse::command::setAcbusGpio,
        0x00, // value
        0x00, // direction
    };
    if (writeData(acbusGpioSetup, sizeof(acbusGpioSetup)) < 0)
    {
        return -EIO;
    }

    return 0;
}

int Mdio::configurePort()
{
    int result = ftdi_usb_reset(&context);
    if (result < 0)
    {
        return -EIO;
    }

    result = ftdi_tcioflush(&context);
    if (result < 0)
    {
        return -EIO;
    }

    result = ftdi_read_data_set_chunksize(&context, 65536);
    if (result < 0)
    {
        return -EIO;
    }

    result = ftdi_write_data_set_chunksize(&context, 65536);
    if (result < 0)
    {
        return -EIO;
    }

    result = ftdi_set_event_char(&context, 0, 0);
    if (result < 0)
    {
        return -EIO;
    }

    result = ftdi_set_error_char(&context, 0, 0);
    if (result < 0)
    {
        return -EIO;
    }

    context.usb_read_timeout = 100;
    context.usb_write_timeout = 5000;

    result = ftdi_set_latency_timer(&context, 1);
    if (result < 0)
    {
        return -EIO;
    }

    result = ftdi_setflowctrl(&context, SIO_RTS_CTS_HS);
    if (result < 0)
    {
        return -EIO;
    }

    if (ftdi_set_bitmode(&context, 0x0, BITMODE_RESET) < 0)
    {
        return -EIO;
    }

    if (ftdi_set_bitmode(&context, 0x0b, BITMODE_MPSSE) < 0)
    {
        return -EIO;
    }

    usleep(50000);
    return 0;
}

int Mdio::setClockDivider(uint16_t clockDivider)
{
    // TCK period = 60MHz / ((1 + clockDivider) * 2)
    const uint8_t command[] = {
        mpsse::command::setClockDivisor,
        static_cast<uint8_t>(clockDivider & 0xff),
        static_cast<uint8_t>((clockDivider >> 8) & 0xff),
    };
    int result = writeData(command, sizeof(command));
    if (result < 0)
    {
        return result;
    }

    return 0;
}

int Mdio::writeData(const uint8_t* data, int size)
{
    int written = ftdi_write_data(&context, data, size);
    return (written == size) ? 0 : -EIO;
}

int Mdio::readData(uint8_t* data, int size)
{
    int remaining = size;
    while (remaining > 0)
    {
        int received = ftdi_read_data(&context, data, remaining);
        if (received < 0)
        {
            return -EIO;
        }
        if (received == 0)
        {
            return -ETIMEDOUT;
        }

        data += received;
        remaining -= received;
    }

    return 0;
}

int Mdio::read(uint8_t phyAddress, uint8_t registerAddress, uint16_t& value)
{
    uint32_t frame =
        makeClause22Frame(clause22::operationRead, phyAddress, registerAddress,
                          clause22::turnaroundRead);

    MpssePacketBuilder packet;
    packet.beginClockData(mpsse::command::clockDataBytesInMsb);
    packet.appendClause22Header(frame);
    const std::size_t readDataOffset = packet.clockDataPayloadSize();
    packet.appendIdle();
    packet.appendIdle();
    packet.endClockData();
    packet.append(mpsse::command::sendImmediate);

    int result = writeData(packet.data(), packet.size());
    if (result < 0)
    {
        return result;
    }

    std::vector<uint8_t> response(packet.clockDataPayloadSize());
    result = readData(response.data(), static_cast<int>(response.size()));
    if (result < 0)
    {
        return result;
    }

    value = (static_cast<uint16_t>(response[readDataOffset]) << 8) |
            response[readDataOffset + 1];
    return 0;
}

int Mdio::write(uint8_t phyAddress, uint8_t registerAddress, uint16_t value)
{
    uint32_t frame =
        makeClause22Frame(clause22::operationWrite, phyAddress, registerAddress,
                          clause22::turnaroundWrite);

    MpssePacketBuilder packet;
    packet.beginClockData(mpsse::command::clockDataBytesOutMsb);
    packet.appendClause22Header(frame);
    packet.appendBigEndian16(value);
    packet.endClockData();
    packet.append(mpsse::command::sendImmediate);

    return writeData(packet.data(), packet.size());
}

} // namespace ftdi_mdio

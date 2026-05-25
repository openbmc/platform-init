// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include <ftdi.h>

#include <cstdint>
#include <string_view>

namespace ftdi_mdio
{

struct DeviceConfig
{
    std::string_view controllerAddress;
    std::string_view location;
    enum ftdi_interface interface = INTERFACE_A;
};

class Mdio
{
  public:
    explicit Mdio(const DeviceConfig& config);
    ~Mdio();

    Mdio(const Mdio&) = delete;
    Mdio& operator=(const Mdio&) = delete;

    int read(uint8_t phyAddress, uint8_t registerAddress, uint16_t& value);
    int write(uint8_t phyAddress, uint8_t registerAddress, uint16_t value);

  private:
    int openDevice(const DeviceConfig& config);
    int initializeOpenedDevice();
    int configurePort();
    int setClockDivider(uint16_t clockDivider);
    int writeData(const uint8_t* data, int size);
    int readData(uint8_t* data, int size);

    struct ftdi_context context = {};
    bool initialized = false;
    bool opened = false;
};

} // namespace ftdi_mdio

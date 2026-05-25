// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "ftdi_mdio.hpp"
#include "gpio.hpp"
#include "mdio.hpp"

#include <systemd/sd-daemon.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <utility>

namespace meta
{

class MdioBackend
{
  public:
    virtual ~MdioBackend() = default;

    virtual int read(uint8_t phy, uint8_t reg, uint16_t& value) = 0;
    virtual int write(uint8_t phy, uint8_t reg, uint16_t value) = 0;
};

template <typename Device>
class MdioBackendAdapter : public MdioBackend
{
  public:
    template <typename... Args>
    explicit MdioBackendAdapter(Args&&... args) :
        mdio(std::forward<Args>(args)...)
    {}

    int read(uint8_t phy, uint8_t reg, uint16_t& value) override
    {
        return mdio.read(phy, reg, value);
    }

    int write(uint8_t phy, uint8_t reg, uint16_t value) override
    {
        return mdio.write(phy, reg, value);
    }

    Device mdio;
};

std::unique_ptr<MdioBackend> createMdioBackend()
{
    int modprobeRet = std::system("/sbin/modprobe mdio-netlink");
    if (modprobeRet)
    {
        std::cerr << "[mdio] failed to load mdio-netlink module\n";
    }

    try
    {
        auto native = std::make_unique<MdioBackendAdapter<mdio::Mdio>>("1*");

        uint16_t value = 0xffff;

        if (native->read(0x00, 0x00, value) == 0 && value != 0xffff)
        {
            std::cerr << "[mdio] using native backend\n";
            return native;
        }

        std::cerr << "[mdio] native probe failed, fallback to FTDI\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "[mdio] native backend exception: " << e.what() << "\n";
    }

    try
    {
        constexpr ftdi_mdio::DeviceConfig ftdiDevice = {
            .controllerAddress = "1e6a1000",
            .location = "1.4",
            .interface = INTERFACE_A,
        };
        auto ftdi =
            std::make_unique<MdioBackendAdapter<ftdi_mdio::Mdio>>(ftdiDevice);

        std::cerr << "[mdio] using FTDI backend\n";

        return ftdi;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[mdio] FTDI backend exception: " << e.what() << "\n";
    }

    throw std::runtime_error("Failed to initialize any MDIO backend");
}

class Mv88e6xxx
{
  public:
    explicit Mv88e6xxx(MdioBackend& mdio) : mdio(mdio) {}

    int readReg(uint8_t phy, uint8_t reg, uint16_t& value, bool print = false)
    {
        int ret = mdio.read(phy, reg, value);

        if (ret || print)
        {
            fprintf(stderr, "    %s: phy=0x%02X reg=0x%02X val=0x%04X %s\n",
                    __func__, phy, reg, value, ret ? "FAILED" : "");
        }

        return ret;
    }

    int writeReg(uint8_t phy, uint8_t reg, uint16_t value, bool print = false)
    {
        int ret = mdio.write(phy, reg, value);

        if (ret || print)
        {
            fprintf(stderr, "    %s: phy=0x%02X reg=0x%02X val=0x%04X %s\n",
                    __func__, phy, reg, value, ret ? "FAILED" : "");
        }

        return ret;
    }

    int readSmiC45(bool ext, uint8_t phy, uint8_t dev, uint16_t reg,
                   uint16_t& value, bool print = false)
    {
        const uint16_t cmdBase =
            Smi::busy | Smi::modeClause45 | Smi::phySelect(ext, phy) |
            Smi::deviceSelect(dev);

        int ret = smiWriteData(reg);
        if (!ret)
        {
            ret = smiCommand(cmdBase | Smi::opClause45WriteAddress);
        }
        if (!ret)
        {
            ret = smiCommand(cmdBase | Smi::opClause45ReadData);
        }
        if (!ret)
        {
            ret = smiReadData(value);
        }

        if (ret || print)
        {
            fprintf(stderr,
                    "    %s: phy=%s/%u dev=%u "
                    "reg=0x%04X val=0x%04X %s\n",
                    __func__, Smi::phyType(ext), phy, dev, reg, value,
                    ret ? "FAILED" : "");
        }

        return ret;
    }

    int writeSmiC45(bool ext, uint8_t phy, uint8_t dev, uint16_t reg,
                    uint16_t value, bool print = false)
    {
        const uint16_t cmdBase =
            Smi::busy | Smi::modeClause45 | Smi::phySelect(ext, phy) |
            Smi::deviceSelect(dev);

        int ret = smiWriteData(reg);
        if (!ret)
        {
            ret = smiCommand(cmdBase | Smi::opClause45WriteAddress);
        }
        if (!ret)
        {
            ret = smiWriteData(value);
        }
        if (!ret)
        {
            ret = smiCommand(cmdBase | Smi::opClause45WriteData);
        }

        if (ret || print)
        {
            fprintf(stderr,
                    "    %s: phy=%s/%u dev=%u "
                    "reg=0x%04X val=0x%04X %s\n",
                    __func__, Smi::phyType(ext), phy, dev, reg, value,
                    ret ? "FAILED" : "");
        }
        return ret;
    }

    int readSmiC22(bool ext, uint8_t phy, uint16_t page, uint8_t reg,
                   uint16_t& value, bool print = false)
    {
        const uint16_t cmdBase =
            Smi::busy | Smi::modeClause22 | Smi::phySelect(ext, phy);
        int ret = smiWriteData(page);
        if (!ret)
        {
            ret = smiCommand(cmdBase | Smi::opClause22Write |
                             Smi::clause22PageRegister);
        }
        if (!ret)
        {
            ret = smiCommand(cmdBase | Smi::opClause22Read |
                             Smi::registerSelect(reg));
        }
        if (!ret)
        {
            ret = smiReadData(value);
        }

        if (ret || print)
        {
            fprintf(stderr,
                    "    %s: phy=%s/%u page=%u "
                    "reg=0x%02X val=0x%04X %s\n",
                    __func__, Smi::phyType(ext), phy, page, reg, value,
                    ret ? "FAILED" : "");
        }
        return ret;
    }

    int writeSmiC22(bool ext, uint8_t phy, uint16_t page, uint8_t reg,
                    uint16_t value, bool print = false)
    {
        const uint16_t cmdBase =
            Smi::busy | Smi::modeClause22 | Smi::phySelect(ext, phy);
        int ret = smiWriteData(page);
        if (!ret)
        {
            ret = smiCommand(cmdBase | Smi::opClause22Write |
                             Smi::clause22PageRegister);
        }
        if (!ret)
        {
            ret = smiWriteData(value);
        }
        if (!ret)
        {
            ret = smiCommand(cmdBase | Smi::opClause22Write |
                             Smi::registerSelect(reg));
        }

        if (ret || print)
        {
            fprintf(stderr,
                    "    %s: phy=%s/%u page=%u "
                    "reg=0x%02X val=0x%04X %s\n",
                    __func__, Smi::phyType(ext), phy, page, reg, value,
                    ret ? "FAILED" : "");
        }
        return ret;
    }

  private:
    struct Smi
    {
        static constexpr uint8_t phyAddress = 0x1c;
        static constexpr uint8_t commandRegister = 0x18;
        static constexpr uint8_t dataRegister = 0x19;

        static constexpr uint16_t busy = 0x8000;
        static constexpr uint16_t modeClause22 = 0x1000;
        static constexpr uint16_t modeClause45 = 0x0000;

        static constexpr uint16_t opClause22Write = 0x0400;
        static constexpr uint16_t opClause22Read = 0x0800;
        static constexpr uint16_t opClause45WriteAddress = 0x0000;
        static constexpr uint16_t opClause45WriteData = 0x0400;
        static constexpr uint16_t opClause45ReadData = 0x0c00;

        static constexpr int waitBusyTimeout = 200;
        static constexpr useconds_t waitBusyDelayUs = 1000;

        static constexpr uint8_t clause22PageRegister = 22;

        static constexpr uint16_t phySelect(bool externalPhy, uint8_t phy)
        {
            return (static_cast<uint16_t>(externalPhy) << 13) |
                   (static_cast<uint16_t>(phy & 0x1f) << 5);
        }

        static constexpr uint16_t deviceSelect(uint8_t device)
        {
            return device & 0x1f;
        }

        static constexpr uint16_t registerSelect(uint8_t reg)
        {
            return reg & 0x1f;
        }

        static const char* phyType(bool externalPhy)
        {
            return externalPhy ? "external" : "internal";
        }
    };

    int waitSmiReady()
    {
        int ret;
        uint16_t value;
        for (int i = 0; i < Smi::waitBusyTimeout; ++i)
        {
            value = 0;
            ret = readReg(Smi::phyAddress, Smi::commandRegister, value);
            if (ret)
            {
                return ret;
            }
            if ((value & Smi::busy) == 0)
            {
                return EXIT_SUCCESS;
            }

            usleep(Smi::waitBusyDelayUs);
        }

        fprintf(stderr, "waitSmiReady: timeout\n");
        return -ETIMEDOUT;
    }

    int smiWriteData(uint16_t value, bool print = false)
    {
        return writeReg(Smi::phyAddress, Smi::dataRegister, value, print);
    }

    int smiReadData(uint16_t& value, bool print = false)
    {
        return readReg(Smi::phyAddress, Smi::dataRegister, value, print);
    }

    int smiCommand(uint16_t cmd, bool print = false)
    {
        int ret = writeReg(Smi::phyAddress, Smi::commandRegister, cmd, print);
        if (!ret)
        {
            ret = waitSmiReady();
        }
        return ret;
    }

    MdioBackend& mdio;
};

int bringup_mv88e6xxx_network()
{
    auto mdio = createMdioBackend();
    Mv88e6xxx sw(*mdio);
    uint16_t value;

    std::cerr << "[mv88e6xxx] bring-up begin\n";

    std::cerr << "[mv88e6xxx] clear PHY detect on Port 0\n";
    int ret = sw.writeReg(0, 0x00, 0x04, true);

    if (!ret)
    {
        std::cerr
            << "[mv88e6xxx] disable Port 0, 9 and 10 before configuring\n";
        ret = sw.writeReg(0, 0x04, 0x7C, true) ||
              sw.writeReg(9, 0x04, 0x7C, true) ||
              sw.writeReg(10, 0x04, 0x7C, true);
    }

    if (!ret)
    {
        std::cerr
            << "[mv88e6xxx] configure external PHY on Port 9 and Port 10 to SGMII mode\n";
        ret = sw.writeSmiC22(1, 0, 18, 20, 0x8001, true) ||
              sw.writeSmiC22(1, 1, 18, 20, 0x8001, true);
    }

    if (!ret)
    {
        std::cerr
            << "[mv88e6xxx] configure switch Port 9 and Port 10 to SGMII mode\n";
        ret = sw.writeReg(9, 0x00, 0x0a, true) ||
              sw.writeReg(10, 0x00, 0x0a, true);
    }

    if (!ret)
    {
        std::cerr << "[mv88e6xxx] force Port 0 link up at 100Mbps\n";
        ret = sw.writeReg(0, 0x01, 0x203D, true);
    }

    if (!ret)
    {
        std::cerr << "[mv88e6xxx] re-enable Port 0, 9 and 10\n";
        ret = sw.writeReg(0, 0x04, 0x7F, true) ||
              sw.writeReg(9, 0x04, 0x7F, true) ||
              sw.writeReg(10, 0x04, 0x7F, true);
    }

    if (!ret)
    {
        // ERRATA 4.7: When changing C_Mode on SERDES port from any mode to
        // 1000BASE-X mode the link may not come up due to invalid 1000BASE-X
        // advertisement
        std::cerr << "[mv88e6xxx] apply ERRATA 4.7 for Port 9 and Port 10\n";
        for (int phy = 9; phy <= 10 && !ret; phy++)
        {
            ret =
                sw.writeSmiC45(false, phy, 4, 0x2004, 0x20, true) ||
                sw.readSmiC45(false, phy, 4, 0x2000, value, true) ||
                sw.writeSmiC45(false, phy, 4, 0x2000, (value | 1 << 15), true);
        }
    }

    if (!ret)
    {
        // ERRATA 4.8: Link may not come up after hardware reset or software
        // reset in 1000BASE-X or SGMII mode
        std::cerr << "[mv88e6xxx] apply ERRATA 4.8 for Port 9 and Port 10\n";
        for (int phy = 9; phy <= 10; phy++)
        {
            ret =
                sw.readSmiC45(false, phy, 4, 0xF074, value, true) ||
                sw.writeSmiC45(false, phy, 4, 0xF074, (value | 1 << 14), true);
        }
    }

    if (!ret)
    {
        std::cerr << "[mv88e6xxx] configure LED on Port 1 through Port 8\n";
        for (int phy = 1; phy <= 8 && !ret; phy++)
        {
            ret = sw.writeReg(phy, 22, 0x8033, true);
        }
    }

    if (!ret)
    {
        std::cerr << "[mv88e6xxx] configure LED on Port 9 and Port 10\n";
        // LED0 Gb Link (off = no link, on = Gb link)
        // LED1 On - Link, Blink - Activity, Off - No Link
        for (int phy = 0; phy <= 1 && !ret; phy++)
        {
            ret = sw.readSmiC22(true, phy, 3, 16, value, true) ||
                  sw.writeSmiC22(true, phy, 3, 16, (value & 0xff00) | 0x0017,
                                 true);
        }
    }

    std::cerr << "[mv88e6xxx] bring-up end\n";
    return ret;
}

int bringup_evt_board()
{
    std::cerr << "[evt_board] bring-up start\n";
    std::cerr
        << "[evt_board]   Set PWRGD signals high to deassert Marvell 88E6393X reset\n";
    gpio::set("FPGA_PWRGD_P1V05_AUX_R", 1);
    gpio::set("FPGA_PWRGD_P5V_AUX_R2", 1);
    gpio::set("FPGA_PWRGD_P12V_AUX_R2", 1);
    gpio::set("FPGA_PWRGD_P1V5_AUX_R", 1);

    std::cerr << "[evt_board]   Reset Marvell 88E6393X\n";
    gpio::set("FPGA_PWRGD_P1V5_AUX_R", 0);
    usleep(12000);
    gpio::set("FPGA_PWRGD_P1V5_AUX_R", 1);

    std::cerr << "[evt_board]   Reset Marvell 88E1512\n";
    gpio::set("FPGA_RST_88E1512_PLD", 0);
    usleep(12000);
    gpio::set("FPGA_RST_88E1512_PLD", 1);
    std::cerr << "[evt_board] bring-up end\n";

    return EXIT_SUCCESS;
}

int init_ventura2()
{
    int ret = bringup_evt_board();
    if (ret)
    {
        return ret;
    }

    ret = bringup_mv88e6xxx_network();
    if (ret)
    {
        return ret;
    }

    sd_notify(0, "READY=1");
    return EXIT_SUCCESS;
}
} // namespace meta

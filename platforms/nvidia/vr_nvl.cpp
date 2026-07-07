// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "gpio.hpp"
#include "i2c.hpp"
#include "sysfs.hpp"
#include "utilities.hpp"

#include <systemd/sd-daemon.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <format>
#include <iostream>
#include <span>
#include <string_view>
#include <vector>

namespace nvidia
{

using namespace std::chrono_literals;

namespace
{

constexpr size_t pdb_bus = 9;
constexpr uint8_t pdb_hsc_probe_addr = 0x10;
constexpr uint8_t pdb_tmp_addr = 0x4e;

constexpr uint8_t pdb_iox_strata_addr = 0x20;
constexpr uint8_t pdb_iox_parsec_addr = 0x75;

constexpr auto hpm_iox_buses = std::to_array<size_t>({2, 7});
constexpr auto hpm_iox_addrs = std::to_array<uint8_t>({0x20, 0x21});

// These signals remain asserted, and 100ms limits polling overhead.
constexpr auto gpio_poll_interval = 100ms;

void wait_asserted_optional(const char* name, std::chrono::seconds timeout,
                            std::string_view reason)
{
    if (gpio::get(name) < 0)
    {
        std::cerr << std::format("{} unresolvable: {} - skipping wait\n", name,
                                 reason);
        return;
    }
    gpio::wait_asserted(name, timeout, gpio_poll_interval);
}

enum class HscVendor
{
    Unknown,
    TI,
    MPS,
    IFX
};

constexpr uint8_t pmbus_mfr_id_reg = 0x99;
constexpr uint8_t pmbus_clear_faults_cmd = 0x03;

constexpr size_t hsc_mfr_id_attempts = 5;
constexpr auto hsc_mfr_id_retry_delay = 100ms;

constexpr auto ti_mfr_id = std::to_array<uint8_t>({0x03, 0x54, 0x49, 0x00});
constexpr auto mps_mfr_id = std::to_array<uint8_t>({0x03, 0x53, 0x50, 0x4d});
constexpr auto ifx_mfr_id = std::to_array<uint8_t>({0x03, 0x49, 0x46, 0x00});

std::vector<uint8_t> read_hsc_mfr_id(uint8_t addr)
{
    try
    {
        i2c::RawDevice dev{pdb_bus, addr};
        // The HSC can transiently return all-zero or all-FF during early
        // boot.
        for (size_t attempt = 0; attempt < hsc_mfr_id_attempts; ++attempt)
        {
            auto r = dev.read_i2c_block(pmbus_mfr_id_reg, 4);
            if (r && r->size() == 4)
            {
                const auto& v = *r;
                bool all_zero =
                    std::ranges::all_of(v, [](uint8_t b) { return b == 0x00; });
                bool all_ff =
                    std::ranges::all_of(v, [](uint8_t b) { return b == 0xff; });
                if (!all_zero && !all_ff)
                {
                    return v;
                }
            }
            // Back off so the transient condition can clear.
            sleep_milliseconds(hsc_mfr_id_retry_delay);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << std::format(
            "PDB HSC MFR_ID read failed @0x{:02x} on bus {}: {}\n", addr,
            pdb_bus, e.what());
    }
    return {};
}

HscVendor detect_hsc_vendor()
{
    auto mfr = read_hsc_mfr_id(pdb_hsc_probe_addr);
    if (mfr.size() != 4)
    {
        std::cerr << "Unable to read PDB HSC MFR_ID\n";
        return HscVendor::Unknown;
    }
    if (std::ranges::equal(mfr, ti_mfr_id))
    {
        std::cerr << "Selected TI PDB HSC vendor\n";
        return HscVendor::TI;
    }
    if (std::ranges::equal(mfr, mps_mfr_id))
    {
        std::cerr << "Selected MPS PDB HSC vendor\n";
        return HscVendor::MPS;
    }
    if (std::ranges::equal(mfr, ifx_mfr_id))
    {
        std::cerr << "Selected IFX PDB HSC vendor\n";
        return HscVendor::IFX;
    }
    std::cerr << "Unknown PDB HSC MFR_ID\n";
    return HscVendor::Unknown;
}

constexpr auto ti_mps_addrs = std::to_array<uint8_t>({0x10, 0x12, 0x14, 0x16});
constexpr auto ifx_addrs = std::to_array<uint8_t>({0x10, 0x1C, 0x1D, 0x1E});

void hsc_write_or_log(i2c::RawDevice& dev, std::span<const uint8_t> data,
                      uint8_t addr, std::string_view what)
{
    if (auto r = dev.write_block(data); !r)
    {
        std::cerr << std::format("HSC {} failed @0x{:02x} on bus {}\n", what,
                                 addr, pdb_bus);
    }
}

void mask_hsc_ti_mps()
{
    constexpr auto mask_alert = std::to_array<uint8_t>({0xd8, 0xff, 0xff});
    constexpr auto clear_fault =
        std::to_array<uint8_t>({pmbus_clear_faults_cmd});
    for (auto addr : ti_mps_addrs)
    {
        try
        {
            i2c::RawDevice dev{pdb_bus, addr};
            hsc_write_or_log(dev, mask_alert, addr, "mask alert (TI/MPS)");
            hsc_write_or_log(dev, clear_fault, addr, "clear fault (TI/MPS)");
        }
        catch (const std::exception& e)
        {
            std::cerr << std::format(
                "HSC RawDevice open failed @0x{:02x}: {}\n", addr, e.what());
        }
    }
    std::cerr << "Completed TI/MPS PDB HSC fault mask & clear\n";
}

void mask_hsc_ifx()
{
    constexpr auto mask_warn = std::to_array<uint8_t>({0xE2, 0x00, 0x00});
    constexpr auto mask_fault = std::to_array<uint8_t>({0xdf, 0x00, 0x00});
    constexpr auto clear_fault =
        std::to_array<uint8_t>({pmbus_clear_faults_cmd});

    for (auto addr : ifx_addrs)
    {
        try
        {
            i2c::RawDevice dev{pdb_bus, addr};
            hsc_write_or_log(dev, mask_warn, addr, "mask warn (IFX)");
            hsc_write_or_log(dev, mask_fault, addr, "mask fault (IFX)");

            auto cfg = dev.read_i2c_block(0xDB, 2);
            if (cfg && cfg->size() == 2)
            {
                uint8_t lo = (*cfg)[0];
                uint8_t new_hi = static_cast<uint8_t>((*cfg)[1] & 0xCF);
                auto gpo_cfg_w = std::to_array<uint8_t>({0xDB, lo, new_hi});
                hsc_write_or_log(dev, gpo_cfg_w, addr,
                                 "GPO_CFG SMBALERT disable (IFX)");
            }
            else
            {
                std::cerr << std::format(
                    "HSC GPO_CFG read failed @0x{:02x} (IFX)\n", addr);
            }

            hsc_write_or_log(dev, clear_fault, addr, "clear fault (IFX)");
        }
        catch (const std::exception& e)
        {
            std::cerr << std::format(
                "HSC RawDevice open failed @0x{:02x}: {}\n", addr, e.what());
        }
    }
    std::cerr << "Completed IFX PDB HSC fault mask & clear\n";
}

void mask_pdb_hsc_faults()
{
    switch (detect_hsc_vendor())
    {
        case HscVendor::TI:
        case HscVendor::MPS:
            mask_hsc_ti_mps();
            break;
        case HscVendor::IFX:
            mask_hsc_ifx();
            break;
        case HscVendor::Unknown:
        default:
            std::cerr << "Skipping vendor-specific PDB HSC fault-mask writes\n";
            break;
    }
}

void configure_pdb_tmp_alert_threshold()
{
    constexpr auto thyst_120c = std::to_array<uint8_t>({0x02, 0x78, 0x00});
    constexpr auto tos_125c = std::to_array<uint8_t>({0x03, 0x7d, 0x00});
    try
    {
        i2c::RawDevice tmp{pdb_bus, pdb_tmp_addr};
        if (auto r = tmp.write_block(thyst_120c); !r)
        {
            std::cerr << "Failed to set PDB TMP THYST=120C\n";
        }
        else
        {
            std::cerr << "Set PDB TMP THYST=120C\n";
        }
        if (auto r = tmp.write_block(tos_125c); !r)
        {
            std::cerr << "Failed to set PDB TMP TOS=125C\n";
        }
        else
        {
            std::cerr << "Set PDB TMP TOS=125C\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << std::format("PDB TMP open failed: {}\n", e.what());
    }
}

void bmc_set_initial_gpio_out()
{
    std::cerr << "Setting initial GPIO state (cold boot)\n";

    gpio::set("GLOBAL_WP_BMC-I", 0);

    mask_pdb_hsc_faults();
    configure_pdb_tmp_alert_threshold();

    gpio::set("BF3_AOC_NCSI_PRSNT_L-I", 0);
    gpio::set("MUX_I2C_ESPI_SEL-O", 0);

    gpio::set("EEDO_LED2-O", 1);
}

// The kernel does not re-probe DTS-declared expanders after standby power
// rises, so late-bind them manually once the rail is up.
void bind_iox(size_t bus, uint8_t addr)
{
    auto path =
        std::format("/sys/bus/i2c/drivers/pca953x/{}-{:04x}", bus, addr);
    if (std::filesystem::exists(path))
    {
        std::cerr << std::format(
            "GPIO Expander {}-{:04x} already bound to pca953x\n", bus, addr);
        return;
    }
    if (auto r = i2c::bind_device(bus, addr, "pca953x"); !r)
    {
        std::cerr << std::format(
            "Failed to bind GPIO Expander {}-{:04x} to pca953x: {}\n", bus,
            addr, r.error().message());
    }
}

void hmc_bypass()
{
    int prsnt = gpio::get("HMC_PRSNT_R-I");
    if (prsnt == 1)
    {
        std::cerr << "HMC present, still use BMC path\n";
    }
    else
    {
        std::cerr << "HMC not present\n";
    }
    gpio::set("BMC_HMC_MUX_SEL-O", 0);
}

// The SMA MCU emulates a CP2112 but enumerates as 0955:CF11, which is
// not in hid-cp2112's id_table, so it lands on hid-generic and needs binding.
void bind_sma_cp2112()
{
    constexpr std::string_view cp2112_driver = "/sys/bus/hid/drivers/cp2112";
    constexpr std::string_view hid_generic_driver =
        "/sys/bus/hid/drivers/hid-generic";

    auto register_result = sysfs::new_id(cp2112_driver, "3 0955 CF11");
    if (!register_result)
    {
        std::cerr << std::format(
            "Failed to register NVIDIA 0955:CF11 with cp2112 driver: {}\n",
            register_result.error().message());
        return;
    }
    std::cerr << "Registered NVIDIA 0955:CF11 with cp2112 driver\n";

    constexpr const char* hid_devices_dir = "/sys/bus/hid/devices/";
    std::error_code ec;
    bool exists = std::filesystem::exists(hid_devices_dir, ec);
    if (ec)
    {
        std::cerr << std::format("bind_sma_cp2112: failed to check {}: {}\n",
                                 hid_devices_dir, ec.message());
        return;
    }
    if (!exists)
    {
        return;
    }

    std::filesystem::directory_iterator entry(hid_devices_dir, ec);
    if (ec)
    {
        std::cerr << std::format("bind_sma_cp2112: failed to open {}: {}\n",
                                 hid_devices_dir, ec.message());
        return;
    }

    const std::filesystem::directory_iterator end;
    while (entry != end)
    {
        std::string devname = entry->path().filename().string();
        if (devname.find(":0955:CF11") != std::string::npos)
        {
            auto unbind_result =
                sysfs::unbind_device(hid_generic_driver, devname);
            if (!unbind_result)
            {
                std::cerr << std::format(
                    "Failed to unbind {} from hid-generic: {}\n", devname,
                    unbind_result.error().message());
            }
            else
            {
                auto bind_result = sysfs::bind_device(cp2112_driver, devname);
                if (!bind_result)
                {
                    std::cerr
                        << std::format("Failed to bind {} to cp2112: {}\n",
                                       devname, bind_result.error().message());
                }
                else
                {
                    std::cerr << std::format(
                        "Migrated {} from hid-generic to cp2112\n", devname);
                }
            }
        }

        entry.increment(ec);
        if (ec)
        {
            std::cerr << std::format(
                "bind_sma_cp2112: failed while iterating {}: {}\n",
                hid_devices_dir, ec.message());
            return;
        }
    }
}

} // namespace

int init_vr_nvl()
{
    std::cerr << "vr-nvl platform init starting\n";

    bind_sma_cp2112();

    int stby_b0 = gpio::get("B0_M0_STBY_POWER_PG-I");
    // Warm path expects Board 0 asserted.
    if (stby_b0 != 1)
    {
        std::cerr << std::format(
            "Cold boot path (B0_M0_STBY_POWER_PG-I = {})\n", stby_b0);

        bmc_set_initial_gpio_out();
    }
    else
    {
        std::cerr << "Warm BMC reset detected; run-power recovery skipped\n";
    }

    bind_iox(pdb_bus, pdb_iox_strata_addr);
    bind_iox(pdb_bus, pdb_iox_parsec_addr);

    gpio::set("PDB_12V_EN_N_R-O", 1);
    sleep_milliseconds(1s);

    gpio::set("STBY_POWER_EN-O", 1);

    if (!gpio::wait_asserted("STBY_POWER_PG-I", 20s, gpio_poll_interval))
    {
        std::cerr << "BMC STBY power good never asserted; aborting\n";
        return EXIT_FAILURE;
    }
    std::cerr << "BMC STBY power good\n";

    gpio::set("HMC_RST_R_L-O", 1);

    gpio::set("PDB_12V_EN_N_R-O", 0);

    if (!gpio::wait_asserted("STBY_PWR_OK-I", 20s, gpio_poll_interval))
    {
        std::cerr << "PDB STBY power good never asserted; aborting\n";
        return EXIT_FAILURE;
    }

    for (auto bus : hpm_iox_buses)
    {
        for (auto addr : hpm_iox_addrs)
        {
            bind_iox(bus, addr);
        }
    }

    hmc_bypass();

    gpio::set("USB_MUX_EN-O", 1);
    gpio::set("BMC_MUX_PI3DP_SEL-O", 0);

    wait_asserted_optional("B0_M0_STBY_POWER_PG-I", 20s,
                           "HPM Board 0 may be absent");
    wait_asserted_optional("B1_M0_STBY_POWER_PG-I", 20s,
                           "HPM Board 1 may be absent");

    wait_asserted_optional("B0_M0_HPM_MCU_OK-I", 10s,
                           "HPM Board 0 MCU may be absent");
    wait_asserted_optional("B1_M0_HPM_MCU_OK-I", 10s,
                           "HPM Board 1 MCU may be absent");

    wait_asserted_optional("B0_M0_CPLD_READY-I", 10s,
                           "HPM Board 0 CPLD may be absent");
    wait_asserted_optional("B1_M0_CPLD_READY-I", 10s,
                           "HPM Board 1 CPLD may be absent");

    sd_notify(0, "READY=1");
    std::cerr << "vr-nvl platform init complete\n";
    pause();
    std::cerr << "Releasing platform\n";
    return EXIT_SUCCESS;
}

} // namespace nvidia

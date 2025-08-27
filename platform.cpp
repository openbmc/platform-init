// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2025 NVIDIA

#include "gpio.hpp"
#include "i2c.hpp"
#include "utilities.hpp"

#include <fcntl.h>
#include <systemd/sd-daemon.h>

#include <CLI/CLI.hpp>
#include <gpiod.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>

using namespace std::chrono_literals;

void init_p2020_gpu_card()
{
    std::cerr << "Initializing GPU card...\n";

    // Init the P2020 gpio expander
    i2c::new_device(14, 0x20, "pca6408");

    // Wait for device to be created
    const auto* device_path = "/sys/bus/i2c/devices/14-0020";
    wait_for_path_to_exist(device_path, 1000ms);

    // Find the GPIO chip number
    int gpiochipint = gpio::find_chip_idx_from_dir(device_path);
    if (gpiochipint < 0)
    {
        return;
    }
    // Set MCU in recovery
    gpio::set_raw(gpiochipint, 3, 1);

    // Reset MCU
    gpio::set_raw(gpiochipint, 4, 0);
    gpio::set_raw(gpiochipint, 4, 1);

    // Switch MUX to MCU
    gpio::set_raw(gpiochipint, 5, 1);
}

bool hmc_is_present()
{
    std::error_code ec;
    bool exists = std::filesystem::exists("/sys/bus/i2c/devices/9-0074", ec);
    if (ec)
    {
        exists = false;
    }
    if (exists)
    {
        std::cerr << "HMC present in platform";
    }
    else
    {
        std::cerr << "HMC not present in platform";
    }
    return exists;
}

int init_nvidia_gb200(bool has_p2020)
{
    // Reset USB hubs
    gpio::set("USB_HUB_RESET_L-O", 0, 10000ms);
    bool hmc_present = hmc_is_present();
    if (!hmc_present)
    {
        gpio::set("SEC_USB2_HUB_RST_L-O", 0, 10000ms);
    }

    sleep_milliseconds(100ms);
    if (!hmc_present)
    {
        gpio::set("SEC_USB2_HUB_RST_L-O", 1);
    }
    //  Write SGPIO_BMC_EN-O=1 to correctly set mux to send SGPIO signals to
    //  FPGA
    gpio::set("SGPIO_BMC_EN-O", 1);

    // Write the bit for BMC without HMC
    gpio::set("HMC_BMC_DETECT-O", static_cast<int>(!hmc_present), 30000ms);

    // Set BMC_EROT_FPGA_SPI_MUX_SEL-O = 1 to enable FPGA to access its EROT
    gpio::set("BMC_EROT_FPGA_SPI_MUX_SEL-O", 1);

    // Enable 12V
    gpio::set("BMC_12V_CTRL-O", 1, 10000ms);

    gpio::set("PWR_BRAKE_L-O", 1);
    gpio::set("SHDN_REQ_L-O", 1);
    gpio::set("SHDN_FORCE_L-O", 1);
    // Hold in reset (asserted) after standby power enabled
    gpio::set("SYS_RST_IN_L-O", 0);

    gpio::Event fpga_ready_wait = gpio::Event("FPGA_READY_BMC-I", 1);
    gpio::Event sec_erot_fpga_rst = gpio::Event("SEC_FPGA_READY_BMC-I", 1);

    // Release FPGA EROT from reset
    gpio::set("EROT_FPGA_RST_L-O", 1);
    gpio::set("SEC_EROT_FPGA_RST_L-O", 1);

    sleep_milliseconds(100ms);

    gpio::set("FPGA_RST_L-O", 1);

    if (fpga_ready_wait.wait() != gpio::EventResult::Asserted)
    {
        std::cerr << "FPGA_READY_BMC-I failed to assert\n";
        // return EXIT_FAILURE;
    }

    if (sec_erot_fpga_rst.wait() != gpio::EventResult::Asserted)
    {
        std::cerr << "SEC_FPGA_READY_BMC-I failed to assert\n";
        // return EXIT_FAILURE;
    }

    // ReInitialize the FPGA connected I2C buses to unstick them and let
    // FruDevice know it can scan for FRUs I2c bus 1
    i2c::rebind_controller("1e78a100");
    // I2c bus 2
    i2c::rebind_controller("1e78a180");

    // Set sgpio signals
    gpio::set("RUN_POWER_EN-O", 1);
    gpio::set("SYS_RST_IN_L-O", 1);
    gpio::set("GLOBAL_WP_BMC-O", 0);

    gpio::set("BMC_READY-O", 1);

    if (has_p2020)
    {
        init_p2020_gpu_card();
    }

    gpio::set("USB_HUB_RESET_L-O", 1);
    if (!hmc_present)
    {
        gpio::set("SEC_USB2_HUB_RST_L-O", 1);
    }

    sd_notify(0, "READY=1");
    std::cerr << "Platform init complete\n";
    pause();
    std::cerr << "Releasing platform\n";

    return EXIT_SUCCESS;
}

int init_nvidia_gb200_base()
{
    return init_nvidia_gb200(false);
}

int init_nvidia_gb200_with_p2020()
{
    return init_nvidia_gb200(true);
}

constexpr std::array<std::pair<std::string_view, int (*)()>, 2> init_functions{
    {{"nvidia-gb200", init_nvidia_gb200_base},
     {"nvidia-gb200-with-p2020", init_nvidia_gb200_with_p2020}}};

int main(int argc, char** argv)
{
    CLI::App app("Platform init CLI");

    app.require_subcommand();

    CLI::App* init_sub =
        app.add_subcommand("init", "Initialize the platform and daemonize");
    std::string platform_name;
    init_sub
        ->add_option("platform_name", platform_name,
                     "Name of the platform to init")
        ->required();
    app.require_subcommand();

    CLI11_PARSE(app, argc, argv)

    const auto* it = std::ranges::find_if(
        init_functions,
        [&platform_name](const std::pair<std::string_view, int (*)()> val) {
            return val.first == platform_name;
        });
    if (it == init_functions.end())
    {
        std::cerr << init_sub->help() << "\n";
        return EXIT_FAILURE;
    }

    return it->second();
}

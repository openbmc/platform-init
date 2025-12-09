#include "gpio.hpp"
#include "i2c.hpp"
#include "utilities.hpp"

#include <systemd/sd-daemon.h>

#include <sdbusplus/asio/connection.hpp>

#include <chrono>
#include <expected>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <unordered_map>

using JsonVariantType =
    std::variant<uint8_t, std::vector<std::string>, std::vector<double>,
                 std::string, int64_t, uint64_t, double, int32_t, uint32_t,
                 int16_t, uint16_t, bool>;
namespace nvidia
{

using steady_clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

void logged_system(std::string_view cmd)
{
    std::cerr << std::format("calling {} \n", cmd);
    int rc = std::system(cmd.data());
    (void)rc;
}

void setup_devmem()
{
    logged_system("mknod /dev/mem c 1 1");
}

void handle_passthrough_registers(bool enable)
{
    static constexpr uint32_t reg = 0x1e6e24bc;
    std::string command;
    if (enable)
    {
        command = std::format("devmem 0x{:x} 32 0x3f000000", reg);
    }
    else
    {
        command = std::format("devmem 0x{:x} 32 0", reg);
    }
    logged_system(command);
}

void wait_for_i2c_ready()
{
    // hpm cpld is at bus 4, address 0x17
    i2c::RawDevice cpld{4, 0x17};
    auto now = steady_clock::now();
    auto end = now + 20min;

    while (steady_clock::now() < end)
    {
        static constexpr uint8_t i2c_ready = 0xf2;
        const auto result = cpld.read_byte(i2c_ready);

        if (!result.has_value())
        {
            std::string err =
                std::format("Unable to communicate with cpld. rc: {}\n",
                            result.error().value());
            std::cerr << err;
            throw std::runtime_error(err);
        }

        if (*result == 1)
        {
            return;
        }

        std::this_thread::sleep_for(std::chrono::seconds{10});
    }

    throw std::runtime_error("Waiting for host timed out!\n");
}

void probe_dev(size_t bus, uint8_t address, std::string_view dev_type)
{
    std::string path =
        std::format("/sys/bus/i2c/devices/i2c-{}/new_device", bus);

    wait_for_path_to_exist(path, std::chrono::milliseconds{1000});

    std::ofstream f{path};
    if (!f.good())
    {
        std::cerr << std::format("Unable to open {}\n", path.c_str());
        std::exit(EXIT_FAILURE);
    }

    f << std::format("{} 0x{:02x}", dev_type, address);
    f.close();

    std::string created_path =
        std::format("/sys/bus/i2c/devices/{}-{:04x}", bus, address);
    wait_for_path_to_exist(created_path, 10ms);
}

void create_i2c_mux(size_t bus, uint8_t address, std::string_view dev_type)
{
    probe_dev(bus, address, dev_type);

    std::string idle =
        std::format("/sys/bus/i2c/devices/{}-{:04x}/idle_state", bus, address);
    std::ofstream idle_f{idle};
    if (!idle_f.good())
    {
        std::string err = std::format("Unable to open {}\n", idle.c_str());
        std::cerr << err;
        throw std::runtime_error(err);
    }

    // -2 is idle-mux-disconnect
    idle_f << -2;
    idle_f.close();
}

size_t get_bus_from_channel(size_t parent_bus, uint8_t address, size_t channel)
{
    std::filesystem::path path =
        std::format("/sys/bus/i2c/devices/{}-{:04x}/channel-{}/i2c-dev/",
                    parent_bus, address, channel);
    int bus = -1;
    std::error_code ec{};
    for (const auto& f : std::filesystem::directory_iterator(path, ec))
    {
        // we expect to see i2c-<bus>, trim and parse everything after the dash
        const std::string& p = f.path().filename().string();
        std::cerr << "Reading from " << p << "\n";
        auto [_, err] = std::from_chars(p.data() + 4, p.data() + p.size(), bus);
        if (err != std::errc{})
        {
            std::string err_s = std::format("Failed to parse {}\n", p);
            std::cerr << err_s;
            throw std::runtime_error(err_s);
        }
    }
    if (bus == -1 || ec)
    {
        std::string err_s =
            std::format("Failed to find a channel at {}\n", path.string());
        std::cerr << err_s;
        throw std::runtime_error(err_s);
    }
    return bus;
}

void bringup_cx8_mcu(size_t bus)
{
    probe_dev(bus, 0x26, "pca9555");
    std::string gpio_p =
        std::format("/sys/bus/i2c/devices/{}-{:04x}/", bus, 0x26);
    int chip_num = gpio::find_chip_idx_from_dir(gpio_p);
    if (chip_num < 0)
    {
        std::cerr << std::format("Failed to find cx8 gpio at {}\n", gpio_p);
        std::exit(EXIT_FAILURE);
    }

    // 14 is the reset pin on the MCU
    // reset pin is active low
    gpio::set_raw(chip_num, 14, 1);
}

void gringup_gpu_sma(size_t bus, size_t channel)
{
    size_t gpu_bus = get_bus_from_channel(bus, 0x72, channel);
    probe_dev(gpu_bus, 0x20, "pca6408");
    std::string gpio_p =
        std::format("/sys/bus/i2c/devices/{}-{:04x}/", gpu_bus, 0x20);
    int chip_num = gpio::find_chip_idx_from_dir(gpio_p);
    if (chip_num < 0)
    {
        std::cerr << std::format("Failed to find gpu gpio {}\n", gpio_p);
        std::exit(EXIT_FAILURE);
    }

    // pin 4 is the reset pin, active low
    // pin 5 engages the telemetry path from the SMA
    gpio::set_raw(chip_num, 5, 1);
    gpio::set_raw(chip_num, 4, 1);
}

void bringup_gpus_on_mcio(size_t bus)
{
    create_i2c_mux(bus, 0x72, "pca9546");

    gringup_gpu_sma(bus, 2);
    gringup_gpu_sma(bus, 3);
}

void bringup_cx8_mcio(size_t mux_addr, size_t channel, bool has_cx8)
{
    size_t bus = get_bus_from_channel(5, mux_addr, channel);
    if (has_cx8)
    {
        bringup_cx8_mcu(bus);
    }
    bringup_gpus_on_mcio(bus);
}

void force_rescan()
{
    auto b = sdbusplus::bus::new_default_system();
    auto m = b.new_method_call("xyz.openbmc_project.EntityManager",
                               "/xyz/openbmc_project/EntityManager",
                               "xyz.openbmc_project.EntityManager", "ReScan");
    b.call(m);
}

void wait_for_frus_to_probe()
{
    std::string path = "/sys/bus/i2c/devices/17-0056";
    wait_for_path_to_exist(path, std::chrono::milliseconds{30 * 1000});

    std::this_thread::sleep_for(std::chrono::seconds{30});
}

int init_nvl32()
{
    setup_devmem();
    gpio::set("BMC_INIT_DONE", 1);
    handle_passthrough_registers(false);
    sd_notify(0, "READY=1");

    wait_for_i2c_ready();
    // we suspect that the CPLD tells us we're ready before
    // we actually are. This sleep stabilizes this discrepency
    std::this_thread::sleep_for(std::chrono::seconds{1});

    create_i2c_mux(5, 0x70, "pca9548");
    create_i2c_mux(5, 0x71, "pca9548");
    create_i2c_mux(5, 0x73, "pca9548");
    create_i2c_mux(5, 0x75, "pca9548");

    bringup_cx8_mcio(0x70, 1, true);
    bringup_cx8_mcio(0x70, 5, false);
    bringup_cx8_mcio(0x73, 3, true);
    bringup_cx8_mcio(0x73, 7, false);

    // there's a weird bug in EntityManager
    // Where Fru devices don't probe automatically
    // We'll wait for the drivers to be probed
    // and then force a rescan
    // we'll follow up with a proper fix
    wait_for_frus_to_probe();

    force_rescan();

    std::cerr << "platform init complete\n";
    pause();
    std::cerr << "Releasing platform\n";

    return EXIT_SUCCESS;
}

} // namespace nvidia

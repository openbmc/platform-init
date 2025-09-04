#include "gpio.hpp"
#include "i2c.hpp"
#include "utilities.hpp"

#include <systemd/sd-daemon.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

namespace nvidia
{

void logged_system(std::string_view cmd)
{
    std::cerr << std::format("calling {} \n", cmd);
    int rc = std::system(cmd.data());
    (void)rc;
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

    while (1)
    {
        static constexpr uint8_t i2c_ready = 0xf2;
        uint8_t result;
        int rc = cpld.read_byte(i2c_ready, result);
        if (rc)
        {
            std::cerr << std::format(
                "Unable to communicate with cpld. rc: {}\n", rc);
            std::exit(EXIT_FAILURE);
        }

        if (result == 1)
        {
            break;
        }

        std::this_thread::sleep_for(std::chrono::seconds{10});
    }
}

void probe_dev(size_t bus, uint8_t address, std::string_view dev_type)
{
    std::filesystem::path path =
        std::format("/sys/bus/i2c/devices/i2c-{}/new_device", bus);

    std::ofstream f{path};
    if (!f.good())
    {
        std::cerr << std::format("Unable to open {}\n", path.c_str());
        std::exit(EXIT_FAILURE);
    }

    f << std::format("{} 0x{:02x}", dev_type, address);
    f.close();

    wait_for_path_to_exist(
        std::format("/sys/bus/i2c/devices/{}-{:04x}", bus, address),
        std::chrono::milliseconds{10});
}

void create_i2c_mux(size_t bus, uint8_t address, std::string_view dev_type)
{
    probe_dev(bus, address, dev_type);

    std::filesystem::path idle =
        std::format("/sys/bus/i2c/devices/{}-{:04x}/idle_state", bus, address);
    std::ofstream idle_f{idle};
    if (!idle_f.good())
    {
        std::cerr << std::format("Unable to open {}\n", idle.c_str());
        std::exit(EXIT_FAILURE);
    }

    // -2 is idle-mux-disconnect
    idle_f << std::format("{}", -2);
    idle_f.close();
}

size_t get_bus_from_channel(size_t parent_bus, uint8_t address, size_t channel)
{
    std::filesystem::path path =
        std::format("/sys/bus/i2c/devices/{}-{:04x}/channel-{}/i2c-dev/",
                    parent_bus, address, channel);
    int bus = -1;
    for (const auto& f : std::filesystem::directory_iterator(path))
    {
        // we expect to see i2c-<bus>, trim and parse everything after the dash
        const std::string& p = f.path().filename().string();
        std::cerr << "Reading from " << p << "\n";
        std::from_chars(p.data() + 4, p.data() + p.size(), bus);
    }
    if (bus == -1)
    {
        std::cerr << std::format("Failed to find a channel at {}\n",
                                 path.string());
        std::exit(EXIT_FAILURE);
    }
    return bus;
}

void bringup_cx8(size_t bus)
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

    gpio::set_raw(chip_num, 14, 1);
}

void bringup_gpu(size_t bus, size_t channel)
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

    gpio::set_raw(chip_num, 5, 1);
    gpio::set_raw(chip_num, 4, 1);
}

void bringup_gpus_on_mcio(size_t bus)
{
    create_i2c_mux(bus, 0x72, "pca9546");

    bringup_gpu(bus, 2);
    bringup_gpu(bus, 3);
}

void bringup_cx8_mcio(size_t mux_addr, size_t channel, bool has_cx8)
{
    size_t bus = get_bus_from_channel(5, mux_addr, channel);
    if (has_cx8)
    {
        bringup_cx8(bus);
    }
    bringup_gpus_on_mcio(bus);
}

void enumerate_mctp(int dev_num)
{
    // TODO: Make this a proper dbus client
    std::string preamble = "busctl call au.com.codeconstruct.MCTP1";
    std::string postamble =
        "au.com.codeconstruct.MCTP1.BusOwner1 AssignEndpoint ay 0";

    std::string cmd =
        std::format("{} /au/com/codeconstruct/mctp1/interfaces/mctpusb{} {}",
                    preamble, dev_num, postamble);
    logged_system(cmd);
}

void wait_for_usb_to_probe()
{
    std::this_thread::sleep_for(std::chrono::seconds{20});
}

int init_nvl32()
{
    handle_passthrough_registers(false);
    sd_notify(0, "READY=1");

    wait_for_i2c_ready();

    create_i2c_mux(5, 0x70, "pca9548");
    create_i2c_mux(5, 0x71, "pca9548");
    create_i2c_mux(5, 0x73, "pca9548");
    create_i2c_mux(5, 0x75, "pca9548");

    bringup_cx8_mcio(0x70, 1, true);
    bringup_cx8_mcio(0x70, 5, false);
    bringup_cx8_mcio(0x73, 3, true);
    bringup_cx8_mcio(0x73, 7, false);

    wait_for_usb_to_probe();
    for (int ctr = 0; ctr < 10; ++ctr)
    {
        enumerate_mctp(ctr);
    }
    std::cerr << "platform init complete\n";
    pause();
    std::cerr << "Releasing platform\n";

    return EXIT_SUCCESS;
}

} // namespace nvidia

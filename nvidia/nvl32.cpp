#include "gpio.hpp"
#include "i2c.hpp"
#include "utilities.hpp"

#include <systemd/sd-daemon.h>

#include <sdbusplus/asio/connection.hpp>

#include <chrono>
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
        uint8_t result;
        int rc = cpld.read_byte(i2c_ready, result);
        if (rc)
        {
            std::string err =
                std::format("Unable to communicate with cpld. rc: {}\n", rc);
            std::cerr << err;
            throw std::runtime_error(err);
        }

        if (result == 1)
        {
            return;
        }

        std::this_thread::sleep_for(std::chrono::seconds{10});
    }

    throw std::runtime_error("Waiting for host timed out!\n");
}

void probe_dev(size_t bus, uint8_t address, std::string_view dev_type)
{
    std::filesystem::path path =
        std::format("/sys/bus/i2c/devices/i2c-{}/new_device", bus);

    wait_for_path_to_exist(path.native(), std::chrono::milliseconds{1000});

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

    std::filesystem::path idle =
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

const char* mctpd_service = "au.com.codeconstruct.MCTP1";
const char* mctp_obj = "/au/com/codeconstruct/mctp1/";
const char* mctp_busowner = "au.com.codeconstruct.MCTP.BusOwner1";
const char* mctp_bridge = "au.com.codeconstruct.MCTP.Bridge1";

template <typename PropertyType>
PropertyType get_property(const char* service, const char* object,
                          const char* interface, const char* property_name)
{
    auto b = sdbusplus::bus::new_default_system();
    auto m = b.new_method_call(service, object,
                               "org.freedesktop.DBus.Properties", "Get");
    m.append(interface, property_name);

    std::variant<PropertyType> t;
    auto reply = b.call(m);

    reply.read(t);
    return std::get<PropertyType>(t);
}

// given a device index
// enumerate the mctp interface
// and give back the eid
uint8_t enumerate_mctp(uint8_t device_idx)
{
    std::vector<uint8_t> address = {};
    std::string obj = std::format(
        "/au/com/codeconstruct/mctp1/interfaces/mctpusb{}", device_idx);

    std::cerr << "calling " << obj << std::endl;

    auto b = sdbusplus::bus::new_default_system();
    auto m = b.new_method_call(mctpd_service, obj.c_str(), mctp_busowner,
                               "AssignEndpoint");
    m.append(address);

    auto reply = b.call(m);

    uint8_t eid;
    int32_t net;
    std::string intf;
    bool probed;
    reply.read(eid, net, intf, probed);

    return eid;
}

// We need to get the pool start and size
std::tuple<uint8_t, uint8_t> get_pool_start_and_size(uint8_t eid)
{
    std::string obj =
        std::format("/au/com/codeconstruct/mctp1/networks/1/endpoints/{}", eid);
    std::cerr << "calling " << obj << std::endl;

    uint8_t poolstart = get_property<uint8_t>(mctpd_service, obj.c_str(),
                                              mctp_bridge, "PoolStart");
    uint8_t poolsize = get_property<uint8_t>(mctpd_service, obj.c_str(),
                                             mctp_bridge, "PoolSize");

    std::cerr << std::format("eid {} has pool start {} and size {}", eid,
                             poolstart, poolsize)
              << std::endl;
    return {poolstart, poolsize};
}

int get_device_from_port_string(std::string_view port_string)
{
    std::filesystem::path path = port_string;
    path /= "net";
    int dev_index = -1;
    auto p = path.native();
    wait_for_path_to_exist(p, std::chrono::milliseconds{20000});

    for (const auto& dir : std::filesystem::directory_iterator(path))
    {
        // this looks something like:
        // /sys/devices/platform/ahb/1e6a3000.usb/usb1/1-1/1-1.2/1-1.2.3/1-1.2.3:1.0/net/mctpusb7
        // we want to extract the final "7"
        std::cerr << "Looking at " << dir.path().native() << std::endl;

        auto f_name = dir.path().filename().native();
        if (f_name.starts_with("mctpusb"))
        {
            std::from_chars(f_name.data() + 7, f_name.data() + f_name.size(),
                            dev_index);
            break;
        }
    }

    if (dev_index == -1)
    {
        std::cerr << std::format("Unable to find an mctpusb net device at {}\n",
                                 path.native());
    }

    std::cerr << "found mctp device index " << dev_index << std::endl;
    return dev_index;
}

bool is_populated(std::string board, std::string name)
{
    std::string obj = std::format(
        "/xyz/openbmc_project/inventory/system/board/{}/{}", board, name);
    std::cerr << "inspecting " << obj << std::endl;
    try
    {
        uint8_t eid = get_property<uint8_t>(
            "xyz.openbmc_project.EntityManager", obj.c_str(),
            "xyz.openbmc_project.Configuration.NvidiaMctpVdm", "StaticEid");
        (void)eid;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

void force_rescan()
{
    auto b = sdbusplus::bus::new_default_system();
    auto m = b.new_method_call("xyz.openbmc_project.EntityManager",
                               "/xyz/openbmc_project/EntityManager",
                               "xyz.openbmc_project.EntityManager", "ReScan");
    b.call(m);
}

void populate_gpu(std::string board, uint8_t eid, std::string name)
{
    if (is_populated(board, name))
    {
        std::cerr << name << " already exists" << std::endl;
        return;
    }

    std::string obj =
        std::format("/xyz/openbmc_project/inventory/system/board/{}", board);

    std::cerr << "calling with " << obj << std::endl;

    std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = start + std::chrono::minutes{3};
    auto b = sdbusplus::bus::new_default_system();
    auto m = b.new_method_call("xyz.openbmc_project.EntityManager", obj.c_str(),
                               "xyz.openbmc_project.AddObject", "AddObject");
    std::unordered_map<std::string, JsonVariantType> param;
    param["Name"] = name;
    param["StaticEid"] = eid;
    param["Type"] = "NvidiaMctpVdm";

    m.append(param);

    do
    {
        auto now = std::chrono::steady_clock::now();
        if (now >= end)
        {
            std::cerr << "Timeout: Failed to add " << obj << std::endl;
            return;
        }
        try
        {
            b.call(m);
            return;
        }
        catch (...)
        {
            std::cerr << "Failed to find " << obj << " trying again"
                      << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds{10});
            continue;
        }
    } while (true);
}

struct bridge_device
{
    std::string name;
    std::string board_name;
};

void bringup_devices()
{
    // There's a lot of hackery going on here
    // This is for handling (as of today) unsupported bridged endpoints
    // The MCU's on this platform act as MCTP bridges
    // We know their absolute USB path through the platform hub, and that's
    // symlinked to a mctp net device So we will start there we also know that
    // each device the USB device is bridging to will always have the same
    // relative ordering
    //  inside of a given pool. This is not a generally true assumption but it
    //  is true for our MCU's
    // So we can put each bridge and is downstream devices through enumeration
    // with mctpd, when we get the response, we know the bridges eid we can then
    // ask mctpd what the pool size and start eid is for the bridge pool. From
    // there we can infer the eid of each bridged device behind it and call
    // AddObject on EntityManager for each board to bring up the requisite nodes
    // beneath it which will allow the rest of the system to start behaving as
    // expected. Once we have real support for bridged eid's, we can and should
    // delete this mess.
    const std::unordered_map<std::string, bridge_device> device_name_map = {
        {"/sys/devices/platform/ahb/1e6a3000.usb/usb1/1-1/1-1.2/1-1.2.1/1-1.2.1:1.0",
         {.name = "GPU_0", .board_name = "Nvidia_RTX_PRO_6000_Blackwell_1"}},
        {"/sys/devices/platform/ahb/1e6a3000.usb/usb1/1-1/1-1.1/1-1.1.2/1-1.1.2.1/1-1.1.2.1:1.0",
         {.name = "GPU_1", .board_name = "Nvidia_RTX_PRO_6000_Blackwell_2"}},
        {"/sys/devices/platform/ahb/1e6a3000.usb/usb1/1-1/1-1.4/1-1.4.1/1-1.4.1:1.0",
         {.name = "GPU_2", .board_name = "Nvidia_RTX_PRO_6000_Blackwell_3"}},
        {"/sys/devices/platform/ahb/1e6a3000.usb/usb1/1-1/1-1.2/1-1.2.2/1-1.2.2:1.0",
         {.name = "GPU_3", .board_name = "Nvidia_RTX_PRO_6000_Blackwell_4"}},
        {"/sys/devices/platform/ahb/1e6a3000.usb/usb1/1-1/1-1.1/1-1.1.4/1-1.1.4.1/1-1.1.4.1:1.0",
         {.name = "GPU_4", .board_name = "Nvidia_RTX_PRO_6000_Blackwell_5"}},
        {"/sys/devices/platform/ahb/1e6a3000.usb/usb1/1-1/1-1.1/1-1.1.2/1-1.1.2.2/1-1.1.2.2:1.0",
         {.name = "GPU_5", .board_name = "Nvidia_RTX_PRO_6000_Blackwell_6"}},
        {"/sys/devices/platform/ahb/1e6a3000.usb/usb1/1-1/1-1.4/1-1.4.2/1-1.4.2:1.0",
         {.name = "GPU_6", .board_name = "Nvidia_RTX_PRO_6000_Blackwell_7"}},
        {"/sys/devices/platform/ahb/1e6a3000.usb/usb1/1-1/1-1.2/1-1.2.3/1-1.2.3:1.0",
         {.name = "CX8_0", .board_name = "NVIDIA_Alon_cx8_Fru"}},
        {"/sys/devices/platform/ahb/1e6a3000.usb/usb1/1-1/1-1.1/1-1.1.4/1-1.1.4.2/1-1.1.4.2:1.0",
         {.name = "GPU_7", .board_name = "Nvidia_RTX_PRO_6000_Blackwell_8"}},
        {"/sys/devices/platform/ahb/1e6a3000.usb/usb1/1-1/1-1.1/1-1.1.2/1-1.1.2.3/1-1.1.2.3:1.0",
         {.name = "CX8_1", .board_name = "NVIDIA_Alon_cx8_Fru"}}};

    for (const auto& [path, dev] : device_name_map)
    {
        std::cerr << "looking at device " << dev.name << std::endl;
        int dev_index = get_device_from_port_string(path);
        if (dev_index < 0)
        {
            std::cerr << std::format(
                "Unable to bring up {} because it doesn't seem to exist\n",
                dev.name);
            continue;
        }

        // enumerate the bridge device
        uint8_t bridge_eid = enumerate_mctp(dev_index);

        auto [pool_start, pool_size] = get_pool_start_and_size(bridge_eid);

        std::this_thread::sleep_for(std::chrono::milliseconds{500});

        // yes this sucks, no I don't like it but we know we'll only have two
        // types of bridged endpoints on this platform and its 9PM the night
        // before it needs to work so we're going to do it *to* it
        if (dev.name.starts_with("GPU"))
        {
            // each GPU has an SMA, as well as a GPU, they both talk over vdm
            // so add both as seperate nodes
            std::cerr << "Adding SMA\n";
            populate_gpu(dev.board_name, bridge_eid, dev.name + "SMA");
            std::cerr << "Adding GPU\n";
            populate_gpu(dev.board_name, pool_start, dev.name);
        }
        else if (dev.name.starts_with("CX8"))
        {
            // TODO: deal with this
            std::cerr << "Skipping CX8's for now\n";
        }
        else
        {
            std::cerr << std::format(
                "Something awful happened with path: {}, name {}\n", path,
                dev.name);
        }
    }
}

void wait_for_frus_to_probe()
{
    std::string path = "/sys/bus/i2c/devices/17-0056";
    wait_for_path_to_exist(path, std::chrono::milliseconds{30 * 1000});

    std::this_thread::sleep_for(std::chrono::seconds{2});
}

int init_nvl32()
{
    setup_devmem();
    handle_passthrough_registers(false);
    sd_notify(0, "READY=1");

    wait_for_i2c_ready();
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
    std::this_thread::sleep_for(std::chrono::seconds{30});
    force_rescan();
    std::this_thread::sleep_for(std::chrono::seconds{1});
    bringup_devices();
    std::cerr << "platform init complete\n";
    pause();
    std::cerr << "Releasing platform\n";

    return EXIT_SUCCESS;
}

} // namespace nvidia

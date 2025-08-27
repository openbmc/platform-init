// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2025 NVIDIA

#include "gpio.hpp"

#include "utilities.hpp"

#include <cstring>
#include <filesystem>
#include <format>
#include <iostream>
#include <unordered_map>

static std::unordered_map<std::string, gpiod::line> io;
using namespace std::chrono_literals;

namespace gpio
{

void set(const char* line_name, int value,
         std::chrono::milliseconds find_timeout)
{
    std::cerr << std::format("{} Request to set to {}\n", line_name, value);
    std::chrono::milliseconds polling_time = 10ms;
    gpiod::line& line = io[line_name];
    if (!line)
    {
        do
        {
            line = gpiod::find_line(line_name);
            if (!line)
            {
                std::cerr << std::format(
                    "{} not found yet, waiting and retrying\n", line_name);

                sleep_milliseconds(polling_time);
                find_timeout -= polling_time;
            }
        } while (!line && find_timeout > 0s);
        if (!line && find_timeout <= 0s)
        {
            std::cerr << std::format("{} Unable to find\n", line_name);
            return;
        }
        try
        {
            line.request({app_name, gpiod::line_request::DIRECTION_OUTPUT, 0},
                         value);
        }
        catch (const std::system_error& e)
        {
            std::cerr << std::format(
                "{} unable to set direction and value {}\n", line_name,
                e.what());
            return;
        }
        // No need to set if the init did it for us
        std::cerr << std::format("{} Set to {}\n", line_name, value);
        return;
    }
    std::cerr << std::format("{} Settingto {}\n", line_name, value);
    line.set_value(value);
}

int get(const char* line_name)
{
    std::cerr << std::format("{} Request to get\n", line_name);

    gpiod::line line = gpiod::find_line(line_name);
    if (!line)
    {
        std::cerr << std::format("{} Set unable to find\n", line_name);
        return -1;
    }
    try
    {
        line.request({app_name, gpiod::line_request::DIRECTION_INPUT, 0});
    }
    catch (const std::system_error& e)
    {
        std::cerr << std::format("{} unable to set {}\n", line_name, e.what());
    }

    int value = line.get_value();
    std::cerr << std::format("{} was {}\n", line_name, value);
    return value;
}

Event::Event(const char* line_name_in, int value_in) :
    line_name(line_name_in), value(value_in)
{
    line = gpiod::find_line(line_name);
    if (!line)
    {
        std::cerr << std::format("{} GpioEvent: Unable to find\n", line_name);
        return;
    }
    int edge = (value != 0) ? ::gpiod::line_request::EVENT_RISING_EDGE
                            : ::gpiod::line_request::EVENT_FALLING_EDGE;

    line.request({app_name, edge, 0});

    int val = line.get_value();
    if (val == value)
    {
        std::cerr << std::format("{} GpioEvent is already {}\n", line_name,
                                 val);
    }
    else
    {
        std::cerr << std::format("GpioEvent created for {}\n", line_name);
    }
}

EventResult Event::wait()
{
    if (!line)
    {
        std::cerr << std::format("Line {} wasn't initialized\n", line_name);
        return EventResult::Error;
    }
    std::cerr << std::format("{}  Waiting to go to {}\n", line_name,
                             (value != 0) ? "assert" : "deassert");
    auto events = line.event_wait(std::chrono::seconds(120));
    if (!events)
    {
        std::cerr << std::format("{} Timeout\n", line_name);
        return EventResult::Timeout;
    }

    std::cerr << std::format("{} Asserted\n", line_name);

    return EventResult::Asserted;
}

void set_raw(unsigned int chip_num, unsigned int bit_num, int value)
{
    std::string syspath = std::format("gpiochip{}", chip_num);
    std::cerr << std::format("Setting gpiochip{} bit {} to {}\n", chip_num,
                             bit_num, value);
    try
    {
        gpiod::chip chip(syspath);
        gpiod::line line = chip.get_line(bit_num);
        line.request({app_name, gpiod::line_request::DIRECTION_OUTPUT, 0},
                     value);
        std::cerr << std::format("gpiochip{} bit {} set to {}\n", chip_num,
                                 bit_num, value);
    }
    catch (const std::system_error& e)
    {
        std::cerr << std::format("Error setting gpiochip{} bit {}: {}\n",
                                 chip_num, bit_num, e.what());
    }
}

int find_chip_idx_from_dir(std::string_view device_path)
{
    std::string gpio_chip;
    for (const auto& entry : std::filesystem::directory_iterator(device_path))
    {
        std::string path = entry.path().string();
        if (path.find("gpiochip") != std::string::npos)
        {
            gpio_chip =
                path.substr(path.find("gpiochip") + std::strlen("gpiochip"));
            break;
        }
    }
    if (gpio_chip.empty())
    {
        std::cerr << "Error: Could not find GPIO chip number\n";
        return -ENOENT;
    }

    std::cerr << "Found GPIO chip: gpiochip" << gpio_chip << "\n";
    unsigned int gpiochipint = 0;
    std::from_chars_result r =
        std::from_chars(&*gpio_chip.begin(), &*gpio_chip.end(), gpiochipint);
    if (r.ec != std::error_code() || r.ptr != &*gpio_chip.end())
    {
        std::cout << "Failed to convert gpiochip\n";
        return -EINVAL;
    }
    return gpiochipint;
}
} // namespace gpio

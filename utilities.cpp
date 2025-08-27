// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2025 NVIDIA

#include "utilities.hpp"

#include <format>
#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>

using namespace std::chrono_literals;

void sleep_milliseconds(std::chrono::milliseconds milliseconds)
{
    std::cerr << std::format("Sleeping for {} milliseconds\n",
                             milliseconds.count());
    std::this_thread::sleep_for(milliseconds);
}

void wait_for_path_to_exist(std::string_view path,
                            std::chrono::milliseconds timeout)
{
    while (true)
    {
        std::error_code ec;
        bool exists = std::filesystem::exists(path, ec);
        if (exists)
        {
            return;
        }
        sleep_milliseconds(1ms);
        timeout -= 1ms;
    }
    std::cerr << std::format("Failed to wait for {} to exist", path);
}

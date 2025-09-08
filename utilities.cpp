// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2025 NVIDIA

#include "utilities.hpp"

#include <chrono>
#include <filesystem>
#include <format>
#include <iostream>
#include <thread>

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
    auto now = std::chrono::steady_clock::now();
    auto end = now + timeout;
    std::cerr << std::format("waiting for {} to exist\n", path);
    while (true)
    {
        std::error_code ec;
        bool exists = std::filesystem::exists(path, ec);
        if (exists)
        {
            return;
        }
        std::this_thread::sleep_for(1ms);

        now = std::chrono::steady_clock::now();
        if (now > end)
        {
            break;
        }
    }
    std::cerr << std::format("Failed to wait for {} to exist\n", path);
}

// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2025 NVIDIA

#pragma once

#include <gpiod.hpp>
#include <chrono>
#include <string>

namespace gpio {
void set(const char* line_name, int value,
              std::chrono::milliseconds find_timeout = std::chrono::milliseconds{1});

void set_raw(unsigned int chip_num, unsigned bit_num, int value);

int get(const char* line_name);

enum class EventResult
{
    Error,
    Asserted,
    Timeout
};

struct Event {
    Event(const char* line_name_in, int value_in);
    EventResult wait();

    gpiod::line line;
    std::string line_name;
    int value;
};

int find_chip_idx_from_dir(std::string_view device_path);

} 

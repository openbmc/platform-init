// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include <gpiod.hpp>

#include <chrono>
#include <string>
#include <string_view>

namespace gpio
{
void set(const char* line_name, int value,
         std::chrono::milliseconds find_timeout = std::chrono::milliseconds{1});

void set_raw(unsigned int chip_num, unsigned bit_num, int value);

int get(const char* line_name);

/**
 * Poll until the named line reads 1.
 *
 * Missing or unreadable lines are retried until timeout.
 *
 * @param line_name GPIO line name.
 * @param timeout Maximum time to wait.
 * @param poll_interval Delay between attempts; must be positive.
 * @return true if the line read 1 before timeout, otherwise false.
 */
[[nodiscard]] bool wait_asserted(const char* line_name,
                                 std::chrono::seconds timeout,
                                 std::chrono::milliseconds poll_interval);

enum class EventResult
{
    Error,
    Asserted,
    Timeout
};

struct Event
{
    Event(const char* line_name_in, int value_in);
    EventResult wait();

    gpiod::line line;
    std::string line_name;
    int value;
};

int find_chip_idx_from_dir(std::string_view device_path);

} // namespace gpio

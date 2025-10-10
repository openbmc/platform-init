// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include <chrono>
#include <string_view>

constexpr static const char* app_name = "platform_init";

void sleep_milliseconds(std::chrono::milliseconds milliseconds);

void wait_for_path_to_exist(std::string_view path,
                            std::chrono::milliseconds timeout);

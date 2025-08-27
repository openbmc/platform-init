// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2025 NVIDIA

#include <fcntl.h>
#include <systemd/sd-daemon.h>

#include <CLI/CLI.hpp>
#include <gpiod.hpp>
#include "nvidia.hpp"

#include <iostream>
#include <array>
#include <string_view>
#include <utility>
#include <algorithm>

constexpr std::array<std::pair<std::string_view, int (*)()>, 2> init_functions{
    {{"nvidia-gb200", nvidia::init_gb200_base},
     {"nvidia-gb200-with-p2020", nvidia::init_gb200_with_p2020}}};

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

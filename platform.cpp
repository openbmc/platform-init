// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "gpio.hpp"
#include "i2c.hpp"
#include "intel.hpp"
#include "nvidia.hpp"
#include "utilities.hpp"

#include <fcntl.h>
#include <systemd/sd-daemon.h>

#include <CLI/CLI.hpp>
#include <gpiod.hpp>

#include <algorithm>
#include <array>
#include <iostream>
#include <string_view>
#include <utility>

constexpr std::array<std::pair<std::string_view, int (*)()>, 5> init_functions{
    {{"intel-acrp", intel::init_acrp},
     {"intel-jcrp", intel::init_jcrp},
     {"nvidia-gb200", nvidia::init_gb200_base},
     {"nvidia-gb200-with-p2020", nvidia::init_gb200_with_p2020},
     {"nvidia-nvl32", nvidia::init_nvl32}}};

void init_sub_callback(std::string& platform_name, bool& success)
{
    const auto* it = std::ranges::find_if(
        init_functions,
        [&platform_name](const std::pair<std::string_view, int (*)()> val) {
            return val.first == platform_name;
        });
    if (it == init_functions.end())
    {
        std::cerr << "no init function";
        return;
    }

    int rc = it->second();
    if (rc != EXIT_SUCCESS)
    {
        std::cerr << "init function failed\n";
        return;
    }

    success = true;
}

int main(int argc, char** argv)
{
    CLI::App app("Platform init CLI");

    app.require_subcommand();

    CLI::App* init_sub =
        app.add_subcommand("init", "Initialize the platform and daemonize");
    std::string platform_name;
    bool success = false;
    init_sub
        ->add_option("platform_name", platform_name,
                     "Name of the platform to init")
        ->required();

    init_sub->callback([&platform_name, &success]() {
        init_sub_callback(platform_name, success);
    });

    app.require_subcommand();
    CLI11_PARSE(app, argc, argv)

    if (!success)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

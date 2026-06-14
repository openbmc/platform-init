// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "gpio.hpp"
#include "i2c.hpp"
#include "intel/platforms.hpp"
#include "nvidia/platforms.hpp"
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

constexpr auto init_functions =
    std::to_array<std::pair<std::string_view, int (*)()>>(
        {{"intel-acrp", intel::init_acrp},
         {"intel-jcrp", intel::init_jcrp},
         {"nvidia-gb200", nvidia::init_gb200_base},
         {"nvidia-gb200-with-p2020", nvidia::init_gb200_with_p2020},
         {"nvidia-nvl32", nvidia::init_nvl32},
         {"nvidia-vr-nvl", nvidia::init_vr_nvl}});

int init_sub_callback(const std::string& platform_name, CLI::App* sub)
{
    const auto* it = std::ranges::find_if(
        init_functions,
        [&platform_name](const std::pair<std::string_view, int (*)()> val) {
            return val.first == platform_name;
        });
    if (it == init_functions.end())
    {
        std::cerr << "No init function for " << platform_name << " platform\n";
        std::cerr << sub->help() << "\n";
        return EXIT_FAILURE;
    }

    int rc = it->second();
    if (rc != EXIT_SUCCESS)
    {
        std::cerr << "init function failed\n";
    }

    return rc;
}

int main(int argc, char** argv)
{
    CLI::App app("Platform init CLI");

    app.require_subcommand();

    CLI::App* init_sub =
        app.add_subcommand("init", "Initialize the platform and daemonize");
    std::string platform_name;
    int rc = EXIT_SUCCESS;
    init_sub
        ->add_option("platform_name", platform_name,
                     "Name of the platform to init")
        ->required();

    init_sub->callback([&platform_name, &rc, init_sub]() {
        rc = init_sub_callback(platform_name, init_sub);
    });

    app.require_subcommand();
    CLI11_PARSE(app, argc, argv)

    return rc;
}

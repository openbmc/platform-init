// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "cmos_reset.hpp"
#include "meta.hpp"

#include <CLI/CLI.hpp>

#include <array>
#include <cstdlib>
#include <string_view>

using std::ranges::find_if;

constexpr std::array<std::pair<std::string_view, void (*)()>, 1>
    reset_functions{{
        {"meta-catalina", meta::catalina_cmos_reset},
    }};

int main(int argc, char** argv)
{
    CLI::App app("CMOS Reset Service CLI");
    argv = app.ensure_utf8(argv);

    std::string platform_name;

    app.add_option("platform_name", platform_name, "Name of the platform")
        ->required(true);

    CLI11_PARSE(app, argc, argv);

    const auto* it = find_if(
        reset_functions,
        [&platform_name](const std::pair<std::string_view, void (*)()> val) {
            return val.first == platform_name;
        });
    if (it == reset_functions.end())
    {
        std::cerr << app.help() << "\n";
        return EXIT_FAILURE;
    }

    cmos::reset::init_service(it->second);

    return EXIT_SUCCESS;
}

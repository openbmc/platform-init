// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "sysfs.hpp"

#include <cerrno>
#include <filesystem>
#include <fstream>

namespace sysfs
{

namespace
{

std::error_code get_write_error()
{
    if (errno != 0)
    {
        return {errno, std::system_category()};
    }
    return std::make_error_code(std::errc::io_error);
}

} // namespace

std::expected<void, std::error_code> write_attribute(std::string_view directory,
                                                     std::string_view attribute,
                                                     std::string_view value)
{
    std::filesystem::path path{directory};
    path /= attribute;

    errno = 0;
    std::ofstream stream(path);
    if (!stream)
    {
        return std::unexpected(get_write_error());
    }

    errno = 0;
    stream << value << std::flush;
    if (!stream)
    {
        return std::unexpected(get_write_error());
    }

    return {};
}

std::expected<void, std::error_code> new_id(std::string_view driver,
                                            std::string_view id)
{
    return write_attribute(driver, "new_id", id);
}

std::expected<void, std::error_code> bind_device(std::string_view driver,
                                                 std::string_view device)
{
    return write_attribute(driver, "bind", device);
}

std::expected<void, std::error_code> unbind_device(std::string_view driver,
                                                   std::string_view device)
{
    return write_attribute(driver, "unbind", device);
}

} // namespace sysfs

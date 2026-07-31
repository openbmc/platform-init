// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "sysfs.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <filesystem>

namespace sysfs
{

std::expected<void, std::error_code> write_attribute(std::string_view directory,
                                                     std::string_view attribute,
                                                     std::string_view value)
{
    if (value.empty())
    {
        return std::unexpected(
            std::make_error_code(std::errc::invalid_argument));
    }

    std::filesystem::path path{directory};
    path /= attribute;

    int fd = ::open(path.c_str(), O_WRONLY);
    if (fd < 0)
    {
        return std::unexpected(std::error_code(errno, std::system_category()));
    }

    ssize_t n = ::write(fd, value.data(), value.size());
    int write_errno = errno;
    ::close(fd);
    if (n < 0)
    {
        return std::unexpected(
            std::error_code(write_errno, std::system_category()));
    }
    if (static_cast<size_t>(n) != value.size())
    {
        return std::unexpected(std::make_error_code(std::errc::io_error));
    }

    return {};
}

std::expected<void, std::error_code> new_id(std::string_view driver_path,
                                            std::string_view id)
{
    return write_attribute(driver_path, "new_id", id);
}

std::expected<void, std::error_code> bind_device(std::string_view driver_path,
                                                 std::string_view device)
{
    return write_attribute(driver_path, "bind", device);
}

std::expected<void, std::error_code> unbind_device(std::string_view driver_path,
                                                   std::string_view device)
{
    return write_attribute(driver_path, "unbind", device);
}

} // namespace sysfs

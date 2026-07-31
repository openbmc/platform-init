// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include <expected>
#include <string_view>
#include <system_error>

namespace sysfs
{

/**
 * Write a value to an attribute in a sysfs directory.
 *
 * @param directory Sysfs directory containing the attribute.
 * @param attribute Attribute file name.
 * @param value Value to write.
 * @return An empty result on success or the write error.
 */
[[nodiscard]] std::expected<void, std::error_code> write_attribute(
    std::string_view directory, std::string_view attribute,
    std::string_view value);

/**
 * Register a device ID through a driver's new_id attribute.
 *
 * @param driver_path Sysfs driver directory.
 * @param id Device ID string to register.
 * @return An empty result on success or the write error.
 */
[[nodiscard]] std::expected<void, std::error_code> new_id(
    std::string_view driver_path, std::string_view id);

/**
 * Bind a device through a driver's bind attribute.
 *
 * @param driver_path Sysfs driver directory.
 * @param device Device identifier to bind.
 * @return An empty result on success or the write error.
 */
[[nodiscard]] std::expected<void, std::error_code> bind_device(
    std::string_view driver_path, std::string_view device);

/**
 * Unbind a device through a driver's unbind attribute.
 *
 * @param driver_path Sysfs driver directory.
 * @param device Device identifier to unbind.
 * @return An empty result on success or the write error.
 */
[[nodiscard]] std::expected<void, std::error_code> unbind_device(
    std::string_view driver_path, std::string_view device);

} // namespace sysfs

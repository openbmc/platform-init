// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include <sdbusplus/bus.hpp>
#include <xyz/openbmc_project/Common/FactoryReset/server.hpp>

namespace host
{
namespace bios
{
namespace reset
{

constexpr const char* softwarePath = "/xyz/openbmc_project/software";
constexpr const char* busName = "xyz.openbmc_project.Host.Updater0";

using BiosResetInherit = sdbusplus::server::object_t<
    sdbusplus::server::xyz::openbmc_project::common::FactoryReset>;

using BiosResetMethod = std::function<void()>;

class BiosReset : public BiosResetInherit
{
  public:
    BiosReset(sdbusplus::bus_t& bus, const char* objPath,
              BiosResetMethod resetMethod = nullptr) :
        BiosResetInherit(bus, objPath), resetMethod(resetMethod)
    {}

    void reset();

  private:
    BiosResetMethod resetMethod;
};

void init_service(BiosResetMethod resetMethod);
} // namespace reset
} // namespace bios
} // namespace host

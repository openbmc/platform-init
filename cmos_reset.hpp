// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include <sdbusplus/bus.hpp>
#include <xyz/openbmc_project/Common/FactoryReset/server.hpp>

namespace cmos
{
namespace reset
{

constexpr const char* softwarePath = "/xyz/openbmc_project/software";
constexpr const char* busName = "xyz.openbmc_project.Host.Updater0";

using CmosResetInherit = sdbusplus::server::object_t<
    sdbusplus::server::xyz::openbmc_project::common::FactoryReset>;

using CmosResetMethod = std::function<void()>;

class CmosReset : public CmosResetInherit
{
  public:
    CmosReset(sdbusplus::bus_t& bus, const char* objPath,
              CmosResetMethod resetMethod = nullptr) :
        CmosResetInherit(bus, objPath), resetMethod(resetMethod)
    {}

    void reset();

  private:
    CmosResetMethod resetMethod;
};

void init_service(CmosResetMethod resetMethod);
} // namespace reset
} // namespace cmos

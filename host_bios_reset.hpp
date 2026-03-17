// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include <sdbusplus/bus.hpp>
#include <xyz/openbmc_project/Common/FactoryReset/server.hpp>
#include <xyz/openbmc_project/ObjectMapper/client.hpp>
#include <xyz/openbmc_project/State/Host/client.hpp>

namespace host
{
namespace bios
{
namespace reset
{

constexpr const char* softwarePath = "/xyz/openbmc_project/software";
constexpr const char* busName = "xyz.openbmc_project.Host.Updater0";

constexpr const char* hostPath = "/xyz/openbmc_project/state/host0";
constexpr const char* hostBusName = "xyz.openbmc_project.State.Host0";

constexpr const char* propertiesInterface = "org.freedesktop.DBus.Properties";

using ObjectMapper = sdbusplus::client::xyz::openbmc_project::ObjectMapper<>;
using HostState = sdbusplus::client::xyz::openbmc_project::state::Host<>;

using BiosResetInherit = sdbusplus::server::object_t<
    sdbusplus::server::xyz::openbmc_project::common::FactoryReset>;

using BiosResetMethod = std::function<void()>;

class BiosReset : public BiosResetInherit
{
  public:
    BiosReset(sdbusplus::bus_t& bus, const char* objPath,
              BiosResetMethod resetMethod = nullptr) :
        BiosResetInherit(bus, objPath), bus(bus), resetMethod(resetMethod)
    {}

    void reset();

  protected:
    sdbusplus::bus_t& bus;

  private:
    BiosResetMethod resetMethod;
    bool isHostRunning();
    void requestHostTransition(HostState::Transition transition);
};

void init_service(BiosResetMethod resetMethod);
} // namespace reset
} // namespace bios
} // namespace host

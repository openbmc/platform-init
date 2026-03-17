// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "activation.hpp"

#include <sdbusplus/bus.hpp>
#include <xyz/openbmc_project/Common/FactoryReset/server.hpp>
#include <xyz/openbmc_project/ObjectMapper/client.hpp>
#include <xyz/openbmc_project/State/Host/client.hpp>

namespace cmos
{
namespace reset
{

constexpr const char* softwarePath = "/xyz/openbmc_project/software";
constexpr const char* busName = "xyz.openbmc_project.Software.Host.Updater0";

constexpr const char* hostPath = "/xyz/openbmc_project/state/host0";
constexpr const char* hostBusName = "xyz.openbmc_project.State.Host0";

constexpr const char* propertiesInterface = "org.freedesktop.DBus.Properties";

using ObjectMapper = sdbusplus::client::xyz::openbmc_project::ObjectMapper<>;
using HostState = sdbusplus::client::xyz::openbmc_project::state::Host<>;

using CmosResetInherit = sdbusplus::server::object_t<
    sdbusplus::server::xyz::openbmc_project::common::FactoryReset>;

using CmosResetMethod = std::function<void()>;

class CmosReset : public CmosResetInherit
{
  public:
    CmosReset(sdbusplus::bus_t& bus, const char* objPath,
              CmosResetMethod resetMethod = nullptr) :
        CmosResetInherit(bus, objPath), bus(bus), resetMethod(resetMethod)
    {}

    virtual void reset();

  private:
    sdbusplus::bus_t& bus;
    CmosResetMethod resetMethod;
    bool isHostRunning();
    void requestHostTransition(HostState::Transition transition);
    std::unique_ptr<activation::BiosResetActivationBlocksTransition>
        activationBlocksTransition = nullptr;
};

void init_service(CmosResetMethod resetMethod);
} // namespace reset
} // namespace cmos

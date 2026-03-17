// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include <sdbusplus/bus.hpp>
#include <xyz/openbmc_project/Common/FactoryReset/aserver.hpp>
#include <xyz/openbmc_project/Common/FactoryReset/event.hpp>
#include <xyz/openbmc_project/Software/ActivationBlocksTransition/aserver.hpp>

namespace cmos
{

constexpr const char* softwarePath = "/xyz/openbmc_project/software";
constexpr const char* busName = "xyz.openbmc_project.Software.Host.Updater0";

using CmosResetMethod =
    std::function<sdbusplus::async::task<bool>(sdbusplus::async::context& ctx)>;

using ResetFailure =
    sdbusplus::error::xyz::openbmc_project::common::FactoryReset::ResetFailure;
using ResetSuccess =
    sdbusplus::event::xyz::openbmc_project::common::FactoryReset::ResetSuccess;

class CmosReset :
    public sdbusplus::aserver::xyz::openbmc_project::common::FactoryReset<
        CmosReset>
{
  public:
    CmosReset(sdbusplus::async::context& ctx, const char* objPath,
              CmosResetMethod resetMethod = nullptr) :
        sdbusplus::aserver::xyz::openbmc_project::common::FactoryReset<
            CmosReset>(ctx, objPath),
        ctx(ctx), resetMethod(resetMethod)
    {}

    auto method_call(reset_t)
    {
        ctx.spawn(startReset());
    };

    sdbusplus::async::task<> startReset();
    sdbusplus::async::task<bool> doReset();

  private:
    sdbusplus::async::context& ctx;
    CmosResetMethod resetMethod;
};

int init_service(CmosResetMethod resetMethod);
} // namespace cmos

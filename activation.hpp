#include <xyz/openbmc_project/Software/ActivationBlocksTransition/server.hpp>

namespace activation
{
using ActivationBlocksTransitionInherit = sdbusplus::server::xyz::
    openbmc_project::software::ActivationBlocksTransition;

class BiosResetActivationBlocksTransition :
    public ActivationBlocksTransitionInherit
{
  public:
    BiosResetActivationBlocksTransition(sdbusplus::bus_t& bus,
                                        const char* objPath) :
        ActivationBlocksTransitionInherit(bus, objPath)
    {}
};
} // namespace activation

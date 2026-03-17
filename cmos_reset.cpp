// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "cmos_reset.hpp"

#include <sdbusplus/bus.hpp>

#include <chrono>
#include <iostream>

namespace cmos

{
namespace reset
{

std::string getSoftwareInstancePath()
{
    return std::format("{}/host0/bios", softwarePath);
}

std::string getHostStatePath()
{
    std::string hostName =
        std::format("{}{}", HostState::namespace_path::host, 0);

    sdbusplus::message::object_path hostStatePath(
        HostState::namespace_path::value);

    hostStatePath /= hostName;

    return hostStatePath.str;
}

std::string getHostStateService(sdbusplus::bus_t& bus)
{
    std::string path = getHostStatePath();

    auto mapperCall = bus.new_method_call(
        ObjectMapper::default_service, ObjectMapper::instance_path,
        ObjectMapper::interface, ObjectMapper::method_names::get_object);

    mapperCall.append(path.c_str(),
                      std::vector<std::string>({HostState::interface}));

    std::map<std::string, std::vector<std::string>> mapperResponse;

    try
    {
        auto mapperResponseMsg = bus.call(mapperCall);
        mapperResponseMsg.read(mapperResponse);
        if (mapperResponse.empty())
        {
            std::cerr << "Invalid response from mapper\n";
            throw std::runtime_error("Error no matching service");
        }
    }
    catch (const sdbusplus::exception_t& e)
    {
        std::cerr << "Error in mapper call\n";
        throw;
    }

    return mapperResponse.begin()->first;
}

bool CmosReset::isHostRunning()
{
    std::string service = getHostStateService(bus);
    std::string path = getHostStatePath();

    auto method = bus.new_method_call(service.c_str(), path.c_str(),
                                      propertiesInterface, "Get");

    method.append(HostState::interface, "CurrentHostState");

    std::variant<std::string> currentHostState;

    try
    {
        auto response = bus.call(method);
        response.read(currentHostState);
    }
    catch (const sdbusplus::exception_t& e)
    {
        std::cerr << "Error fetching current host state\n";
        throw;
    }

    if (std::get<std::string>(currentHostState).empty())
    {
        std::cerr << "Error reading CurrentHostState property\n";
        throw std::runtime_error("Error reading CurrentHostState property");
    }

    return std::get<std::string>(currentHostState) !=
           HostState::convertHostStateToString(HostState::HostState::Off);
}

void CmosReset::requestHostTransition(HostState::Transition transition)
{
    std::string requested = HostState::convertTransitionToString(transition);
    std::cerr << "Requesting " << requested << " host transition\n";

    std::string service = getHostStateService(bus);
    std::string path = getHostStatePath();
    std::variant<std::string> requestedTransition = requested;

    auto method = bus.new_method_call(service.c_str(), path.c_str(),
                                      propertiesInterface, "Set");

    method.append(HostState::interface,
                  HostState::property_names::requested_host_transition,
                  requestedTransition);

    bus.call_noreply(method);
    return;
}

void CmosReset::reset()
{
    if (resetMethod)
    {
        if (isHostRunning())
        {
            std::cerr << "Host is running. Requesting host transiton to off\n";
            requestHostTransition(HostState::Transition::Off);
            std::chrono::seconds timeout(10);

            while (timeout.count() != 0)
            {
                timeout--;

                if (!isHostRunning())
                {
                    std::cerr << "Transition successful. Starting reset\n";
                    break;
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            if (timeout.count() == 0)
            {
                std::cerr << "Unable to transition host to off\n";
                return;
            }
        }

        activationBlocksTransition =
            std::make_unique<activation::BiosResetActivationBlocksTransition>(
                bus, getSoftwareInstancePath().c_str());

        activationBlocksTransition->emit_added();

        resetMethod();

        activationBlocksTransition.reset(nullptr);

        requestHostTransition(HostState::Transition::On);

        return;
    }
    return;
}

void init_service(CmosResetMethod resetMethod)
{
    sdbusplus::bus_t bus = sdbusplus::bus::new_default();

    sdbusplus::server::manager_t objManager(bus, softwarePath);

    CmosReset service{bus, getSoftwareInstancePath().c_str(), resetMethod};

    bus.request_name(cmos::reset::busName);

    bus.process_loop();
}
} // namespace reset
} // namespace cmos

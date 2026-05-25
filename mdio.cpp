// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "mdio.hpp"

#include <glob.h>
#include <linux/genetlink.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace mdio
{

using namespace mdio::abi;

static constexpr auto attributeIndex(NetlinkAttribute attribute)
{
    return static_cast<int>(attribute);
}

static constexpr auto commandValue(GenericNetlinkCommand command)
{
    return static_cast<uint8_t>(command);
}

static constexpr uint64_t makeRegisterArgument(uint16_t registerAddress)
{
    return (static_cast<uint64_t>(ArgumentMode::registerReference) << 16) |
           registerAddress;
}

static constexpr uint64_t makeImmediateArgument(uint16_t value)
{
    return (static_cast<uint64_t>(ArgumentMode::immediate) << 16) | value;
}

static constexpr Instruction makeInstruction(
    Operation operation, uint64_t argument0, uint64_t argument1,
    uint64_t argument2)
{
    return Instruction{
        .op = static_cast<uint64_t>(operation),
        .reserved = 0,
        .arg0 = argument0,
        .arg1 = argument1,
        .arg2 = argument2,
    };
}

Mdio::Mdio(const char* busPattern)
{
    bus = findFirstBus(busPattern);
    if (bus.empty())
        throw std::runtime_error(
            "failed to find mdio bus: " + std::string(busPattern));

    if (openSocket() < 0)
        throw std::runtime_error("failed to open netlink socket");

    if (initFamilyId() < 0)
        throw std::runtime_error("failed to init mdio netlink family id");
}

Mdio::~Mdio()
{
    if (socket != nullptr)
    {
        mnl_socket_close(socket);
        socket = nullptr;
    }
}

int Mdio::openSocket()
{
    if (socket != nullptr)
        return 0;

    socket = mnl_socket_open(NETLINK_GENERIC);
    if (socket == nullptr)
        return -errno;

    if (mnl_socket_bind(socket, 0, MNL_SOCKET_AUTOPID) < 0)
    {
        int socketError = -errno;
        mnl_socket_close(socket);
        socket = nullptr;
        return socketError;
    }

    return 0;
}

int Mdio::query(nlmsghdr* netlinkHeader, mnl_cb_t callback, void* data)
{
    const uint32_t seqNum = ++seq;
    const unsigned portId = mnl_socket_get_portid(socket);
    netlinkHeader->nlmsg_seq = seqNum;

    if (mnl_socket_sendto(socket, netlinkHeader, netlinkHeader->nlmsg_len) < 0)
        return -errno;

    int ret;
    do
    {
        ret = mnl_socket_recvfrom(socket, msgBuffer, sizeof(msgBuffer));
        if (ret <= 0)
            break;
        ret = mnl_cb_run(msgBuffer, ret, seqNum, portId, callback, data);
    } while (ret > 0);

    return (ret == 0) ? 0 : ret;
}

int Mdio::parseAttr(const nlattr* attr, void* data)
{
    auto** table = static_cast<const nlattr**>(data);
    table[mnl_attr_get_type(attr)] = attr;
    return MNL_CB_OK;
}

int Mdio::familyIdCallback(const nlmsghdr* netlinkHeader, void* data)
{
    auto* self = static_cast<Mdio*>(data);

    nlattr* attrs[CTRL_ATTR_MAX + 1] = {};
    mnl_attr_parse(netlinkHeader, sizeof(genlmsghdr), parseAttr, attrs);

    if (attrs[CTRL_ATTR_FAMILY_ID] == nullptr)
        return MNL_CB_ERROR;

    self->familyId = mnl_attr_get_u16(attrs[CTRL_ATTR_FAMILY_ID]);
    return MNL_CB_OK;
}

int Mdio::initFamilyId()
{
    auto* netlinkHeader = mnl_nlmsg_put_header(msgBuffer);
    netlinkHeader->nlmsg_type = GENL_ID_CTRL;
    netlinkHeader->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;

    auto* genericNetlinkHeader = static_cast<genlmsghdr*>(
        mnl_nlmsg_put_extra_header(netlinkHeader, sizeof(genlmsghdr)));
    genericNetlinkHeader->cmd = CTRL_CMD_GETFAMILY;
    genericNetlinkHeader->version = 1;

    mnl_attr_put_u16(netlinkHeader, CTRL_ATTR_FAMILY_ID, GENL_ID_CTRL);
    mnl_attr_put_strz(netlinkHeader, CTRL_ATTR_FAMILY_NAME, "mdio");

    return query(netlinkHeader, familyIdCallback, this);
}

int Mdio::transferCallback(const nlmsghdr* netlinkHeader, void* data)
{
    auto* context = static_cast<TransferContext*>(data);

    nlattr* attrs[attributeIndex(NetlinkAttribute::max) + 1] = {};
    mnl_attr_parse(netlinkHeader, sizeof(genlmsghdr), parseAttr, attrs);

    if (attrs[attributeIndex(NetlinkAttribute::error)] != nullptr)
        context->error = static_cast<int>(
            mnl_attr_get_u32(attrs[attributeIndex(NetlinkAttribute::error)]));

    /* write operations produce no data attribute */
    if (attrs[attributeIndex(NetlinkAttribute::data)] == nullptr)
    {
        int ret =
            context->callback(nullptr, 0, context->error, context->userData);
        if (ret != 0)
            context->error = ret;
        return (ret == 0) ? MNL_CB_OK : MNL_CB_ERROR;
    }

    int payloadLength = mnl_attr_get_payload_len(
                            attrs[attributeIndex(NetlinkAttribute::data)]) /
                        sizeof(uint32_t);
    auto* payload = static_cast<uint32_t*>(
        mnl_attr_get_payload(attrs[attributeIndex(NetlinkAttribute::data)]));

    int ret = context->callback(payload, payloadLength, context->error,
                                context->userData);
    if (ret != 0)
        context->error = ret;
    return (ret == 0) ? MNL_CB_OK : MNL_CB_ERROR;
}

int Mdio::transfer(Instruction* instructions, int instructionCount,
                   TransferCallback callback, void* userData)
{
    TransferContext context{
        .callback = callback,
        .userData = userData,
        .error = 0,
    };

    auto* netlinkHeader = mnl_nlmsg_put_header(msgBuffer);
    netlinkHeader->nlmsg_type = familyId;
    netlinkHeader->nlmsg_flags =
        NLM_F_REQUEST; /* no ACK: response carries error code */

    auto* genericNetlinkHeader = static_cast<genlmsghdr*>(
        mnl_nlmsg_put_extra_header(netlinkHeader, sizeof(genlmsghdr)));
    genericNetlinkHeader->cmd = commandValue(GenericNetlinkCommand::transfer);
    genericNetlinkHeader->version = 1;

    mnl_attr_put_strz(netlinkHeader, attributeIndex(NetlinkAttribute::busId),
                      bus.c_str());
    mnl_attr_put(netlinkHeader, attributeIndex(NetlinkAttribute::program),
                 instructionCount * sizeof(*instructions), instructions);
    mnl_attr_put_u16(netlinkHeader, attributeIndex(NetlinkAttribute::timeout),
                     1000);

    int ret = query(netlinkHeader, transferCallback, &context);
    return (context.error != 0) ? context.error : ret;
}

int Mdio::readCallback(uint32_t* data, int length, int error, void* userData)
{
    if (error != 0)
        return error;
    if (length != 1)
        return -EINVAL;

    *static_cast<uint16_t*>(userData) = static_cast<uint16_t>(data[0]);
    return 0;
}

int Mdio::writeCallback([[maybe_unused]] uint32_t* data, int length, int error,
                        [[maybe_unused]] void* userData)
{
    if (error != 0)
        return error;
    if (length != 0)
        return -EINVAL;
    return 0;
}

int Mdio::read(uint8_t phyAddress, uint8_t registerAddress, uint16_t& value)
{
    Instruction instructions[] = {
        makeInstruction(Operation::read, makeImmediateArgument(phyAddress),
                        makeImmediateArgument(registerAddress),
                        makeRegisterArgument(0)),
        makeInstruction(Operation::emit, makeRegisterArgument(0), 0, 0),
    };
    return transfer(instructions, 2, readCallback, &value);
}

int Mdio::write(uint8_t phyAddress, uint8_t registerAddress, uint16_t value)
{
    Instruction instructions[] = {
        makeInstruction(Operation::write, makeImmediateArgument(phyAddress),
                        makeImmediateArgument(registerAddress),
                        makeImmediateArgument(value)),
    };
    return transfer(instructions, 1, writeCallback, nullptr);
}

std::string Mdio::findFirstBus(const char* pattern)
{
    char globPattern[128] = {};
    snprintf(globPattern, sizeof(globPattern), "/sys/class/mdio_bus/%s",
             pattern);

    glob_t matches = {};
    if (glob(globPattern, 0, nullptr, &matches) != 0 || matches.gl_pathc == 0)
    {
        globfree(&matches);
        return {};
    }

    std::string result = matches.gl_pathv[0] + strlen("/sys/class/mdio_bus/");
    globfree(&matches);
    return result;
}

} // namespace mdio

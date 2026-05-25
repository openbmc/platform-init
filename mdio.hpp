// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include <libmnl/libmnl.h>

#include <cstdint>
#include <string>

namespace mdio
{

// C++ reimplementation of mdio-tools/include/linux/mdio-netlink.h ABI
namespace abi
{

// Subset of mdio-netlink ABI used by this wrapper.
// Keep numeric values aligned with mdio-tools/include/linux/mdio-netlink.h.

// Generic Netlink commands (MDIO_GENL_*)
enum class GenericNetlinkCommand : uint8_t
{
    transfer = 1,
};

// Netlink attributes (MDIO_NLA_*)
enum class NetlinkAttribute : uint8_t
{
    busId = 1,
    timeout = 2,
    program = 3,
    data = 4,
    error = 5,
    max = error,
};

// Bytecode opcodes (MDIO_NL_OP_*)
enum class Operation : uint8_t
{
    read = 1,
    write = 2,
    emit = 8,
};

// Argument modes (MDIO_NL_ARG_*)
enum class ArgumentMode : uint8_t
{
    registerReference = 1,
    immediate = 2,
};

struct Instruction
{
    uint64_t op:8;
    uint64_t reserved:2;
    uint64_t arg0:18;
    uint64_t arg1:18;
    uint64_t arg2:18;
};

} // namespace abi

class Mdio
{
  public:
    explicit Mdio(const char* busPattern);
    ~Mdio();

    Mdio(const Mdio&) = delete;
    Mdio& operator=(const Mdio&) = delete;

    int read(uint8_t phyAddress, uint8_t registerAddress, uint16_t& value);
    int write(uint8_t phyAddress, uint8_t registerAddress, uint16_t value);

  private:
    using TransferCallback = int (*)(uint32_t* data, int length, int error,
                                     void* userData);

    struct TransferContext
    {
        TransferCallback callback;
        void* userData;
        int error;
    };

    int openSocket();
    int initFamilyId();
    int query(nlmsghdr* netlinkHeader, mnl_cb_t callback, void* data);
    int transfer(abi::Instruction* instructions, int instructionCount,
                 TransferCallback callback, void* userData);

    static std::string findFirstBus(const char* pattern);
    static int parseAttr(const nlattr* attr, void* data);
    static int familyIdCallback(const nlmsghdr* netlinkHeader, void* data);
    static int transferCallback(const nlmsghdr* netlinkHeader, void* data);
    static int readCallback(uint32_t* data, int length, int error,
                            void* userData);
    static int writeCallback(uint32_t* data, int length, int error,
                             void* userData);

    alignas(NLMSG_ALIGNTO) char msgBuffer[0x1000] = {};
    std::string bus;
    mnl_socket* socket = nullptr;
    uint16_t familyId = 0;
    uint32_t seq = 0;
};

} // namespace mdio

#ifndef DELTA_HPP
#define DELTA_HPP

#include <cstdint>

// uint32_t is 0x[Address(4byte), SubIndex(2byte), typeSizeInBits(2byte)]
typedef struct PACKED
{
    uint16_t control_word;   // 0x6040
    int32_t target_position; // 0x607A PPU
    int32_t target_velocity; // 0x60FF 0.1 rpm
    int16_t target_torque;   // 0x6071 0.1 %
} out_deltab3_t;
constexpr uint32_t rx_mapping[] = {0x60400010, 0x607A0020, 0x60FF0020, 0x60710010};
constexpr uint32_t rx_mapping_count = sizeof(rx_mapping) / sizeof(uint32_t);

typedef struct PACKED
{
    uint16_t status_word;    // 0x6041
    int32_t actual_position; // 0x6064 PPU
    int16_t actual_velocity; // 0x606C 0.1 rpm
} in_deltab3_t;
constexpr uint32_t tx_mapping[] = {0x60410010, 0x60640020, 0x606C0020};
constexpr uint32_t tx_mapping_count = sizeof(tx_mapping) / sizeof(uint32_t);

#endif

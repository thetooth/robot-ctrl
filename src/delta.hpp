#ifndef DELTA_HPP
#define DELTA_HPP

#include <cstdint>

#include "ethercat.h"
#include "common.hpp"

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

namespace Delta
{
    [[maybe_unused]] static int PO2SOconfig(uint16_t slave)
    {
        // Set operational mode
        Common::wkc += Common::Write8(slave, 0x6060, 0, 0x0);

        // Set interpolation values
        Common::wkc += Common::Write8(slave, 0x60C2, 1, 2);
        Common::wkc += Common::Write8(slave, 0x60C2, 2, -3);

        const auto mapPDO = [&](const uint16_t PDO_map, const uint32_t *data, const uint8_t dataSize, const uint32_t SM_map) -> int
        {
            int wkc = 0;
            // Unmap previous registers, setting 0 in PDO_MAP subindex 0
            uint32_t zeroU32 = 0;
            wkc += ec_SDOwrite(slave, PDO_map, 0, FALSE, sizeof(zeroU32), &zeroU32, EC_TIMEOUTRXM);
            // Modify mapping, setting register address in PDO's subindexes from 0x1A00:01
            for (uint32_t i = 0; i < dataSize; i++)
            {
                uint8_t subIndex = static_cast<uint8_t>(i + 1);
                wkc += ec_SDOwrite(slave, SM_map, subIndex, FALSE, sizeof(data[i]), &data[i], EC_TIMEOUTRXM);
            }
            // Enable mapping by setting number of registers in PDO_MAP subindex 0
            wkc += ec_SDOwrite(slave, PDO_map, 0, FALSE, sizeof(dataSize), &dataSize, EC_TIMEOUTRXM);
            // Set PDO mapping to SM
            uint8_t zeroU8 = 0;
            // Unmap previous mappings, setting 0 in SM_MAP subindex 0
            wkc += ec_SDOwrite(slave, SM_map, 0, FALSE, sizeof(zeroU8), &zeroU8, EC_TIMEOUTRXM);
            // Write first mapping (PDO_map) address in SM_MAP subindex 1
            wkc += ec_SDOwrite(slave, SM_map, 1, FALSE, sizeof(PDO_map), &PDO_map, EC_TIMEOUTRXM);
            uint8_t pdoMapSize = 1;
            // Save mapping count in SM (here only one PDO_MAP)
            wkc += ec_SDOwrite(slave, SM_map, 0, FALSE, sizeof(pdoMapSize), &pdoMapSize, EC_TIMEOUTRXM);
            return wkc;
        };

        printf("MAP PDO ");
        Common::wkc += mapPDO(0x1A00, tx_mapping, tx_mapping_count, 0x1C13);
        Common::wkc += mapPDO(0x1600, rx_mapping, rx_mapping_count, 0x1C12);

        printf("slave_id: %x\n", slave);
        return 0;
    }
}

#endif

#ifndef DELTA_HPP
#define DELTA_HPP

#include <cstdint>

#include "ethercat.h"
#include "pdo.hpp"
#include "spdlog/spdlog.h"

namespace Delta
{
    // uint32_t is 0x[Address(4byte), SubIndex(2byte), typeSizeInBits(2byte)]
    typedef struct PACKED
    {
        uint16_t control_word;    // 0x6040
        int32_t target_position;  // 0x607A PPU
        int32_t target_velocity;  // 0x60FF 0.1 rpm
        int16_t target_torque;    // 0x6071 0.1 %
        uint32_t digital_outputs; // 0x60FE sub 1 [16-19 DO1-DO4]
    } rx_t;
    constexpr uint32_t rx_mapping[] = {0x60400010, 0x607A0020, 0x60FF0020, 0x60710010, 0x60FE0120};
    constexpr uint32_t rx_mapping_count = sizeof(rx_mapping) / sizeof(uint32_t);

    typedef struct PACKED
    {
        uint16_t status_word;    // 0x6041
        int32_t actual_position; // 0x6064 PPU
        int32_t actual_velocity; // 0x606C 0.1 rpm
        int16_t actual_torque;   // 0x6077 0.1 %
        int32_t following_error; // 0x60F4 PPU
        uint16_t error_code;     // 0x603F
        uint32_t digital_inputs; // 0x60FD [0 Neg Limit][1 Pos Limit][2 Homing switch][16-19 DI1-DI4]
    } tx_t;
    constexpr uint32_t tx_mapping[] = {0x60410010, 0x60640020, 0x606C0020, 0x60770010,
                                       0x60F40020, 0x603F0010, 0x60FD0020};
    constexpr uint32_t tx_mapping_count = sizeof(tx_mapping) / sizeof(uint32_t);

    int PO2SOconfig(uint16_t slave);

    class PDO : public Drive::PDO
    {
      public:
        tx_t *in;
        rx_t *out;

        PDO(int slaveID)
        {
            in = (Delta::tx_t *)ec_slave[slaveID].inputs;
            out = (Delta::rx_t *)ec_slave[slaveID].outputs;
        }

        uint16_t getStatusWord() const;
        int32_t getActualPosition() const;
        int32_t getActualVelocity() const;
        int16_t getActualTorque() const;
        int32_t getFollowingError() const;
        uint16_t getErrorCode() const;
        uint32_t getDigitalInputs() const;
        bool getEmergencyStop() const;

        void setControlWord(uint16_t value);
        void setTargetPosition(int32_t value);
        void setTargetVelocity(int32_t value);
        void setTargetTorque(int16_t value);
        void setDigitalOutputs(uint32_t value);
    };
} // namespace Delta

#endif

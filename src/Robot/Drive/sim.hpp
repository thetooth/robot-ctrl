#ifndef DRIVE_SIM_HPP
#define DRIVE_SIM_HPP

#include <cstdint>

#include "pdo.hpp"
#include "spdlog/spdlog.h"

namespace Sim
{
    class PDO : public Drive::PDO
    {
      public:
        uint16_t status_word;
        int32_t following_error;
        uint32_t digital_inputs;

        uint16_t control_word;
        int32_t target_position;
        int32_t target_velocity;
        int16_t target_torque;
        uint32_t digital_outputs;

        uint16_t getStatusWord() const;
        int32_t getActualPosition() const;
        int32_t getActualVelocity() const;
        int16_t getActualTorque() const;
        int32_t getFollowingError() const;
        uint32_t getDigitalInputs() const;

        void setControlWord(uint16_t value);
        void setTargetPosition(int32_t value);
        void setTargetVelocity(int32_t value){};
        void setTargetTorque(int16_t value){};
        void setDigitalOutputs(uint32_t value){};
    };
} // namespace Sim

#endif

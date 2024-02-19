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
        PDO()
            : status_word(0), following_error(0), digital_inputs(0), control_word(0), target_position(0),
              target_velocity(0), target_torque(0), digital_outputs(0)
        {
            spdlog::info("Simulated drive created");
        }

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
        uint16_t getErrorCode() const;
        uint32_t getDigitalInputs() const;
        bool getEmergencyStop() const;

        void setControlWord(uint16_t value);
        void setTargetPosition(int32_t value);
        void setTargetVelocity([[maybe_unused]] int32_t value){};
        void setTargetTorque([[maybe_unused]] int16_t value){};
        void setDigitalOutputs([[maybe_unused]] uint32_t value){};
    };
} // namespace Sim

#endif

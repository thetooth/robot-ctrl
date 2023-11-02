#ifndef CAN_COEFSM_HPP
#define CAN_COEFSM_HPP

#include <chrono>
#include <stdio.h>
#include <thread>

#include "spdlog/spdlog.h"

using namespace std::literals::chrono_literals;

/// \enum CANOpenState The different states of a DS402-compliant device.
enum class CANOpenState
{
    OFF,
    SAFE_RESET,
    PREPARE_TO_SWITCH_ON,
    SWITCH_ON,
    ON,
    HOMING_COMPLETE,
    FAULT
};

/// \enum CANOpenCommand The command to send to the CANOpen state machine.
enum class CANOpenCommand
{
    NONE,
    ENABLE,
    DISABLE,
    HOME
};

// status stores the possible CAN states
// For more infos, see DS402, page 35
// CANopen DS402  0x6041 Statusword
namespace status
{
    namespace masks
    {
        // Status bits
        uint16_t const STATUS_MASK = 0x7F;
        uint16_t const READY_TO_SWITCH_ON = 1U << 0;
        uint16_t const SWITCHED_ON = 1U << 1;
        uint16_t const OPERATION_ENABLE = 1U << 2;
        uint16_t const FAULT_MODE = 1U << 3;
        uint16_t const VOLTAGE_ENABLED = 1U << 4;
        uint16_t const QUICK_STOP = 1U << 5;
        uint16_t const SWITCH_ON_DISABLED = 1U << 6;
        uint16_t const WARNING = 1U << 7;
        uint16_t const REMOTE = 1U << 9;
        uint16_t const TARGET_REACHED = 1U << 10;
        uint16_t const INTERNAL_LIMIT_ACTIVE = 1U << 11;
        uint16_t const SETPOINT_ACKNOWLEDGE = 1U << 12;
    } // namespace masks

    namespace value
    {
        /// \brief Some status word values corresponding to CANOpen states.
        uint16_t const OFF_STATE = masks::SWITCH_ON_DISABLED;
        uint16_t const READY_TO_SWITCH_ON_STATE = masks::VOLTAGE_ENABLED | masks::READY_TO_SWITCH_ON;
        uint16_t const ON_STATE = masks::QUICK_STOP | masks::VOLTAGE_ENABLED | masks::OPERATION_ENABLE |
                                  masks::SWITCHED_ON | masks::READY_TO_SWITCH_ON;
        uint16_t const HOMING_COMPLETE_STATE = masks::QUICK_STOP | masks::VOLTAGE_ENABLED | masks::OPERATION_ENABLE |
                                               masks::SWITCHED_ON | masks::READY_TO_SWITCH_ON | masks::TARGET_REACHED |
                                               masks::SETPOINT_ACKNOWLEDGE;
        uint16_t const DISABLED_STATE =
            masks::QUICK_STOP | masks::VOLTAGE_ENABLED | masks::SWITCHED_ON | masks::READY_TO_SWITCH_ON;
        uint16_t const FAULT_STATE = masks::FAULT_MODE;
    } // namespace value
} // namespace status

// control stores the possible control word values
// For more infos, see DS402, page 35
// CANopen DS402  0x6040 Controlword
namespace control
{
    namespace word
    {
        uint16_t const SHUTDOWN = 0x0006U;
        uint16_t const SWITCH_ON_OR_DISABLE_OPERATION = 0x0007U;
        uint16_t const ENABLE_OPERATION = 0x000FU;
        uint16_t const ENABLE_OPERATION_IB = 0x000FU;
        uint16_t const FAULT_RESET = 0x0080U; // 0x00C0U;
        uint16_t const DISABLE_VOLTAGE = 0x0000U;
        uint16_t const QUICK_STOP = 0x0002U;
        uint16_t const SET_ABS_POINT_NOBLEND = 0x001FU;
        uint16_t const SET_POINT_RESET = 0x000FU;
    } // namespace word

    // controlmode stores the available CAN control mode (also called "Mode of Operation")
    // For more infos, see DS402, page 37
    enum CANOpenControlMode
    {
        NO_MODE = -1,       // Default value to mean "error occurred"
        POSITION = 1,       // Profiled position (point to point) mode
        VELOCITY = 3,       // Profiled velocity mode
        TORQUE = 4,         // Profiled torque mode
        POSITION_CYCLIC = 8 // Direct position control without ramps
    };

} // namespace control

// Timeouts to prevent blocking the state machine in case of failure
constexpr std::chrono::nanoseconds MOTOR_RESET_DELAY = 10ms;
constexpr std::chrono::nanoseconds MOTOR_INIT_TIMEOUT = 1s;

class CoEFSM
{
  public:
    void update(uint16_t status_word);
    void setCommand(CANOpenCommand command)
    {
        command_ = command;
    };
    uint16_t getControlWord()
    {
        return control_word_;
    }
    bool compareState(CANOpenState desired)
    {
        return motor_state_ == desired;
    }
    std::string to_string() const
    {
        return "";
    }

  private:
    CANOpenCommand command_ = CANOpenCommand::NONE;
    CANOpenState motor_state_ = CANOpenState::OFF;
    std::chrono::nanoseconds start_motor_timestamp_ = 0ns;
    uint16_t control_word_ = 0;
    uint16_t status_word_ = 0;
};

#endif

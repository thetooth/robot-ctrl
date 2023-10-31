#include "CoEFSM.h"

void CoEFSM::update(uint16_t status_word)
{
    status_word_ = status_word;

    switch (command_)
    {
    case CANOpenCommand::ENABLE:
    {
        // DEBUG_PRINT("CANOpenCommand ENABLE\n");
        switch (motor_state_)
        {
        case CANOpenState::OFF:
        {
            // DEBUG_PRINT("CANOpenState OFF\n");
            start_motor_timestamp_ = std::chrono::system_clock::now().time_since_epoch();
            control_word_ = control::word::FAULT_RESET;
            motor_state_ = CANOpenState::SAFE_RESET;
            break;
        }
        case CANOpenState::SAFE_RESET:
        {
            // DEBUG_PRINT("CANOpenState SAFE_RESET\n");
            control_word_ = control::word::SHUTDOWN;
            if ((std::chrono::system_clock::now().time_since_epoch() - start_motor_timestamp_) > MOTOR_RESET_DELAY)
            {
                motor_state_ = CANOpenState::PREPARE_TO_SWITCH_ON;
            }
            break;
        }
        case CANOpenState::PREPARE_TO_SWITCH_ON:
        {
            // DEBUG_PRINT("CANOpenState PREPARE_TO_SWITCH_ON\n");
            control_word_ = control::word::SWITCH_ON_OR_DISABLE_OPERATION;
            if ((status_word_ & status::value::READY_TO_SWITCH_ON_STATE) == status::value::READY_TO_SWITCH_ON_STATE)
            {
                motor_state_ = CANOpenState::SWITCH_ON;
            }
            break;
        }
        case CANOpenState::SWITCH_ON:
        {
            // DEBUG_PRINT("CANOpenState SWITCH_ON\n");
            control_word_ = control::word::ENABLE_OPERATION;
            if ((status_word_ & status::value::ON_STATE) == status::value::ON_STATE)
            {
                motor_state_ = CANOpenState::ON;
            }
            break;
        }
        default:
        case CANOpenState::ON:
            // DEBUG_PRINT("ON status achieved\n");
            // Reset command now that the desired state has been reached.
            command_ = CANOpenCommand::NONE;
            break;
        case CANOpenState::FAULT:
            DEBUG_PRINT("CANOpenState FAULT\n");
            break;
        }

        // Time out, motor start failed
        if ((motor_state_ != CANOpenState::ON) and (motor_state_ != CANOpenState::FAULT) and (motor_state_ != CANOpenState::OFF) and ((std::chrono::system_clock::now().time_since_epoch() - start_motor_timestamp_) > MOTOR_INIT_TIMEOUT))
        {
            DEBUG_PRINT("Can't enable motor: timeout, start again from OFF state.\n");
            motor_state_ = CANOpenState::OFF;
        }
        break;
    }
    case CANOpenCommand::DISABLE:
    {
        // DEBUG_PRINT("CANOpenCommand DISABLE\n");
        control_word_ = control::word::DISABLE_VOLTAGE;
        if ((status::value::OFF_STATE & status_word_) == status::value::OFF_STATE)
        {
            motor_state_ = CANOpenState::OFF;

            // Reset command now that the desired state has been reached.
            command_ = CANOpenCommand::NONE;
        }
        break;
    }
    case CANOpenCommand::NONE:
    default:
    {
        // DEBUG_PRINT("CANOpenState NONE\n");
        break;
    }
    }
}

#include "CoE.hpp"

void CANOpen::FSM::update(uint16_t status_word)
{
    status_word_ = status_word;

    switch (command_)
    {
    case CANOpenCommand::ENABLE: {
        switch (motor_state_)
        {
        case CANOpenState::OFF: {
            spdlog::trace("CANOpenState OFF");
            start_motor_timestamp_ = std::chrono::system_clock::now().time_since_epoch();
            control_word_ = control::word::FAULT_RESET;
            motor_state_ = CANOpenState::SAFE_RESET;
            break;
        }
        case CANOpenState::SAFE_RESET: {
            spdlog::trace("CANOpenState SAFE_RESET");
            control_word_ = control::word::SHUTDOWN;
            if ((std::chrono::system_clock::now().time_since_epoch() - start_motor_timestamp_) > MOTOR_RESET_DELAY)
            {
                motor_state_ = CANOpenState::PREPARE_TO_SWITCH_ON;
            }
            break;
        }
        case CANOpenState::PREPARE_TO_SWITCH_ON: {
            spdlog::trace("CANOpenState PREPARE_TO_SWITCH_ON");
            control_word_ = control::word::SWITCH_ON_OR_DISABLE_OPERATION;
            if ((status_word_ & status::value::READY_TO_SWITCH_ON_STATE) == status::value::READY_TO_SWITCH_ON_STATE)
            {
                motor_state_ = CANOpenState::SWITCH_ON;
            }
            break;
        }
        case CANOpenState::SWITCH_ON: {
            spdlog::trace("CANOpenState SWITCH_ON");
            control_word_ = control::word::ENABLE_OPERATION;
            if ((status_word_ & status::value::ON_STATE) == status::value::ON_STATE)
            {
                motor_state_ = CANOpenState::ON;
            }
            break;
        }
        default:
        case CANOpenState::ON:
            spdlog::trace("ON status achieved");
            // Reset command now that the desired state has been reached.
            command_ = CANOpenCommand::NONE;
            break;
        case CANOpenState::FAULT:
            spdlog::error("CANOpenState FAULT");
            break;
        }

        // Time out, motor start failed
        if ((motor_state_ != CANOpenState::ON) and (motor_state_ != CANOpenState::FAULT) and
            (motor_state_ != CANOpenState::OFF) and
            ((std::chrono::system_clock::now().time_since_epoch() - start_motor_timestamp_) > MOTOR_INIT_TIMEOUT))
        {
            spdlog::warn("Can't enable motor: timeout, start again from OFF state.");
            motor_state_ = CANOpenState::OFF;
        }
        break;
    }
    case CANOpenCommand::DISABLE: {
        control_word_ = control::word::DISABLE_VOLTAGE;
        if ((status::value::OFF_STATE & status_word_) == status::value::OFF_STATE)
        {
            motor_state_ = CANOpenState::OFF;
            spdlog::trace("OFF_STATE status achieved");
            // Reset command now that the desired state has been reached.
            command_ = CANOpenCommand::NONE;
        }
        break;
    }
    case CANOpenCommand::HOME: {
        switch (motor_state_)
        {
        case CANOpenState::ON:
            control_word_ = control::word::SET_ABS_POINT_NOBLEND;
            if ((status_word_ & status::value::HOMING_COMPLETE_STATE) == status::value::HOMING_COMPLETE_STATE)
            {
                spdlog::trace("HOMING_COMPLETE status achieved");
                motor_state_ = CANOpenState::HOMING_COMPLETE;
            }

            break;
        case CANOpenState::HOMING_COMPLETE:
            control_word_ = control::word::ENABLE_OPERATION;
            // Reset command now that the desired state has been reached.
            command_ = CANOpenCommand::NONE;
            break;
        default:
        case CANOpenState::FAULT:
            spdlog::error("CANOpenState FAULT");
            break;
        }
    }
    case CANOpenCommand::NONE:
    default: {
        break;
    }
    }
}

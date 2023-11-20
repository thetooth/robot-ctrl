#include "drive.hpp"

//! @brief Update CoE state machine
//!
//! NOTE: Trajectory must be computed and sent to the drive PRIOR to entering
//! CSP, not doing this results in PDO target position being zero at activation
//! and drive attempting to leave low earth orbit. For safety the target is set
//! to the current position every cycle.
void Drive::Motor::update()
{
    CANOpen::FSM::update(InPDO->status_word);
    if (compareState(CANOpenState::FAULT) && !fault)
    {
        lastFault = fmt::format("Drive {} CoE entered {} state", slaveID, CANOpen::FSM::to_string());
        spdlog::error(lastFault);
        fault = true;
    }
    OutPDO->control_word = CANOpen::FSM::getControlWord();
    OutPDO->target_position = InPDO->actual_position;
}

bool Drive::Motor::move(double target)
{
    if (fault)
    {
        return fault;
    }
    auto current = InPDO->actual_position / gearRatio;
    if (std::abs(target - current) > 100)
    {
        fault = true;
        lastFault =
            fmt::format("Target position {} from current position {} exceeds dynamic capabilities", target, current);
        spdlog::error(lastFault);
        return fault;
    }
    if (target < minPosition || target > maxPosition)
    {
        fault = true;
        lastFault = fmt::format("Target position {} outside soft limits", target);
        spdlog::error(lastFault);
        return fault;
    }
    OutPDO->target_position = target * gearRatio;
    return fault;
}

double Drive::Motor::getPosition()
{
    return InPDO->actual_position / gearRatio;
}

double Drive::Motor::getVelocity()
{
    return InPDO->actual_velocity / (10.0 * 3.44);
}

int Drive::Motor::setModeOfOperation(CANOpen::control::mode value)
{
    return ec_SDOwrite(slaveID, 0x6060, 0, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
}

int Drive::Motor::setHomingOffset(int32_t value)
{
    return ec_SDOwrite(slaveID, 0x607C, 0, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
}

int Drive::Motor::faultReset()
{
    fault = false;
    lastFault = "I'm OK";
    return ec_SDOwrite(slaveID, 0x6040, 0, FALSE, sizeof(CANOpen::control::word::FAULT_RESET),
                       &CANOpen::control::word::FAULT_RESET, EC_TIMEOUTRXM);
}
#include "drive.hpp"

//! @brief Update CoE state machine
//!
//! Updates the CANOpen state machine of the drive. If the drive enters a FAULT state and it was
//! not previously in a fault state, an error is logged.
//!
//! NOTE: Trajectory must be computed and sent to the drive PRIOR to entering CSP, not doing this
//! results in PDO target position being zero at activation and drive attempting to leave low
//! earth orbit. For safety the target is set to the current position every cycle.
void Drive::Motor::update()
{
    CANOpen::FSM::update(pdo->getStatusWord());
    if (compareState(CANOpenState::FAULT) && !fault)
    {
        lastFault = fmt::format("Drive {} CoE entered {} state", slaveID, CANOpen::FSM::to_string());
        spdlog::error(lastFault);
        fault = true;
    }
    auto errorCode = pdo->getErrorCode();
    if (errorCode != 0 && !fault)
    {
        lastFault = fmt::format("Drive {} error code {}", slaveID, errorCode);
        spdlog::error(lastFault);
        fault = true;
    }
    pdo->setControlWord(CANOpen::FSM::getControlWord());
    pdo->setTargetPosition(pdo->getActualPosition());
}

//! @brief Move the drive to a target position
//!
//! This function moves the drive to the specified target position. It first checks if the drive
//! is in a fault state. If it is, the function returns immediately. Then, it calculates the current
//! position of the motor and compares it with the target position. If the difference exceeds a certain
//! threshold, or if the target position is outside the soft limits, an error message is logged and
//! no movement is performed.
//!
//! @param target The target position to move the motor to
//! @return True if the motor is in a fault state, false otherwise
bool Drive::Motor::move(double target)
{
    if (fault)
    {
        return fault;
    }
    auto current = pdo->getActualPosition() / positionRatio;
    if (std::abs(target - current) > 300)
    {
        fault = true;
        lastFault = fmt::format("Target deviation", target, current);
        spdlog::error(lastFault);
        return fault;
    }
    if (target < minPosition || target > maxPosition)
    {
        fault = true;
        lastFault = fmt::format("Outside soft limits", target);
        spdlog::error(lastFault);
        return fault;
    }
    pdo->setTargetPosition(target * positionRatio);
    return fault;
}

//! @brief Get the current position of the drive
//!
//! This function returns the current position of the drive in degrees.
//!
//! @return The current position of the drive
double Drive::Motor::getPosition() const
{
    return pdo->getActualPosition() / positionRatio;
}

//! @brief Get the current velocity of the drive
//!
//! This function returns the current velocity of the drive in degrees/s.
//!
//! @return The current velocity of the drive
double Drive::Motor::getVelocity() const
{
    return pdo->getActualVelocity() / velocityRatio;
}

//! @brief Get the current torque of the drive
//!
//! This function returns the current torque of the drive in %.
//!
//! @return The current torque of the drive
double Drive::Motor::getTorque() const
{
    return pdo->getActualTorque() / 10.0;
}

//! @brief Get the current following error of the drive
//!
//! This function returns the current following error of the drive in degrees.
//!
//! @return The current following error of the drive
double Drive::Motor::getFollowingError() const
{
    return pdo->getFollowingError() / positionRatio;
}

//! @brief Get the current error code of the drive
//!
//! This function returns the current error code of the drive.
//!
//! @return The current error code of the drive
uint16_t Drive::Motor::getErrorCode() const
{
    return pdo->getErrorCode();
}

//! @brief Set the mode of operation for the drive
//!
//! This function sets the mode of operation for the drive.
//!
//! @param value The mode of operation to set
//! @return Current working counter
int Drive::Motor::setModeOfOperation(CANOpen::control::mode value)
{
    return ec_SDOwrite(slaveID, 0x6060, 0, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
}

//! @brief Set the homing offset for the drive
//!
//! This function sets the homing offset for the drive. The homing offset is used to
//! adjust the position reported by the motor after homing.
//!
//! @param value The homing offset to set
//! @return Current working counter
int Drive::Motor::setHomingOffset(int32_t value)
{
    auto final = int32_t(value * positionRatio);
    return ec_SDOwrite(slaveID, 0x607C, 0, FALSE, sizeof(final), &final, EC_TIMEOUTRXM);
}

//! @brief Set the torque limit for the drive in %
int Drive::Motor::setTorqueLimit(double value)
{
    value = std::max(std::min(value, 100.0), 0.0);
    auto final = uint16_t(value * 10);
    return ec_SDOwrite(slaveID, 0x6072, 0, FALSE, sizeof(final), &final, EC_TIMEOUTRXM);
}

//! @brief Set the following window for the drive in degrees, outside of this window AL009
//! is set and the drive requires reset.
int Drive::Motor::setFollowingWindow(double value)
{
    value = std::max(value, 0.0);
    auto final = uint32_t(value * positionRatio);
    return ec_SDOwrite(slaveID, 0x6065, 0, FALSE, sizeof(final), &final, EC_TIMEOUTRXM);
}

//! @brief Reset the fault state of the drive
//!
//! This function resets the fault state of the drive. And sends a fault reset command to the motor.
//!
//! @return Current working counter
int Drive::Motor::faultReset()
{
    fault = false;
    lastFault = "OK";
    return ec_SDOwrite(slaveID, 0x6040, 0, FALSE, sizeof(CANOpen::control::word::FAULT_RESET),
                       &CANOpen::control::word::FAULT_RESET, EC_TIMEOUTRXM);
}

//! @brief Convert a drive to JSON
void Drive::to_json(json &j, const Motor &m)
{
    j = json{
        {"slaveID", m.slaveID},
        {"fault", m.fault},
        {"lastFault", m.lastFault},
        {"followingError", m.getFollowingError()},
        {"actualTorque", m.getTorque()},
        {"statusWord", m.pdo->getStatusWord()},
        {"controlWord", m.getControlWord()},
        {"errorCode", m.getErrorCode()},
    };
}

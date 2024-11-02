#include "fsm.hpp"

//! @brief Configure homing parameters
//!
//! This function configures the homing parameters for the robot.
//!
void Robot::FSM::configureHoming()
{
    Arm.setModeOfOperation(CANOpen::control::mode::HOME);

    // Arm.setCommand(CANOpenCommand::HOME);
}

//! @brief Home the robot
//!
//! This function homes the robot.
//!
//! @return bool
bool Robot::FSM::homing()
{
    static int axisToHome = 0;
    switch (axisToHome)
    {
    case 0:
        J1.setCommand(CANOpenCommand::HOME);
        if (J1.compareState(CANOpenState::HOMING_COMPLETE))
        {
            axisToHome++;
        }
        break;
    case 1:
        J2.setCommand(CANOpenCommand::HOME);
        if (J2.compareState(CANOpenState::HOMING_COMPLETE))
        {
            axisToHome++;
        }
        break;
    case 2:
        J3.setCommand(CANOpenCommand::HOME);
        if (J3.compareState(CANOpenState::HOMING_COMPLETE))
        {
            axisToHome++;
        }
        break;
    case 3:
        J4.setCommand(CANOpenCommand::HOME);
        if (J4.compareState(CANOpenState::HOMING_COMPLETE))
        {
            axisToHome++;
        }
        break;
    default:
        eventLog.Debug("Homing complete");
        needsHoming = false;

        break;
    }
    if (axisToHome > 3)
    {
        axisToHome = 0;
        return true;
    }

    return false;
}
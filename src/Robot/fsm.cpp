/**
 * @file fsm.cpp
 * @brief Implementation of the FSM (Finite State Machine) class for controlling a robot.
 */
#include "fsm.hpp"

void Robot::FSM::update()
{
    Arm.update();

    switch (next)
    {
    default:
    case Idle:
        if (reset)
        {
            reset = false;
            needsHoming = true;
            next = Reset;
        }

        if (estop && run)
        {
            eventLog.Info("Entering run mode");
            next = Reset;
        }

        if (!estop)
        {
            needsHoming = true;
            next = Halt;
        }
        break;
    case Reset:
        Arm.faultReset();

        {
            auto pendingErrorCode = false;
            for (auto &&drive : Arm.drives)
            {
                if (drive->getErrorCode() != 0)
                {
                    auto msg =
                        fmt::format("Drive {} has pending error code {:#x}", drive->slaveID, drive->getErrorCode());
                    eventLog.Warning(msg);
                    pendingErrorCode = true;
                }
            }

            if (!pendingErrorCode)
            {
                eventLog.Info("Fault reset complete");

                if (run)
                {
                    next = Start;
                }
                else
                {
                    next = Idle;
                }
            }
        }

        break;
    case Halt:
        Arm.setModeOfOperation(CANOpen::control::mode::NO_MODE);
        Arm.setCommand(CANOpenCommand::DISABLE);

        next = Halting;
        break;
    case Halting:
        if (J1.compareState(CANOpenState::OFF) && J2.compareState(CANOpenState::OFF))
        {
            next = Idle;
        }
        break;
    case Start:
        Arm.setCommand(CANOpenCommand::ENABLE);

        next = Starting;
        break;
    case Starting:
        if (J1.compareState(CANOpenState::ON) && J2.compareState(CANOpenState::ON))
        {
            if (needsHoming)
            {
                eventLog.Info("Entered ON state, enter homing");
                next = Home;
            }
            else
            {
                eventLog.Info("Entered ON state, enter tracking");
                next = Track;
            }
        }
        if (!estop || !run)
        {
            next = Halt;
        }
        break;
    case Home:
        Arm.setModeOfOperation(CANOpen::control::mode::HOME);
        J1.setHomingOffset(-235);
        J2.setHomingOffset(145);
        Arm.setCommand(CANOpenCommand::HOME);

        next = Homing;
    case Homing: {
        auto homingResult =
            J1.compareState(CANOpenState::HOMING_COMPLETE) && J2.compareState(CANOpenState::HOMING_COMPLETE);
        if (homingResult)
        {
            eventLog.Info("Homing complete");
            needsHoming = false;
            run = false;
            next = Halt;
        }
        if (!estop || !run)
        {
            next = Halt;
        }
        break;
    }
    case Track:
        Arm.setModeOfOperation(CANOpen::control::mode::POSITION_CYCLIC);
        next = Tracking;
        break;
    case Tracking: {
        auto trackingResult = tracking();
        if (!estop || !run || trackingResult)
        {
            eventLog.Info("Tracking interrupted EStop: " + std::to_string(estop) + " Run: " + std::to_string(run) +
                          " Tracking: " + std::to_string(trackingResult));
            inSync = false;
            next = Halt;
        }
        break;
    }
    case Path:
        Arm.setModeOfOperation(CANOpen::control::mode::POSITION_CYCLIC);
        next = Pathing;
        break;
    case Pathing:
        if (!estop || !run)
        {
            eventLog.Info("Pathing interrupted EStop: " + std::to_string(estop) + " Run: " + std::to_string(run));
            next = Halt;
        }
    }
}

std::string Robot::FSM::to_string() const
{
    switch (next)
    {
    case Idle:
        return "Idle";
    case Halt:
        return "Halt";
    case Halting:
        return "Halting";
    case Start:
        return "Start";
    case Starting:
        return "Starting";
    case Home:
        return "Home";
    case Homing:
        return "Homing";
    case Track:
        return "Track";
    case Tracking:
        return "Tracking";
    default:
        return "[Unknown State]";
    }
}

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
            // needsHoming = true;
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
    case Reset: {
        auto pendingErrorCode = false;
        for (auto &&drive : Arm.drives)
        {
            if (drive->fault || drive->getErrorCode() != 0)
            {
                if (drive->getErrorCode() != 0)
                {
                    eventLog.Warning(
                        fmt::format("J{} has pending error code {:#x}", drive->slaveID, drive->getErrorCode()));
                }
                else
                {
                    eventLog.Warning(fmt::format("J{} has pending fault: {}", drive->slaveID, drive->lastFault));
                }
                pendingErrorCode = true;
            }
        }

        if (!pendingErrorCode)
        {
            eventLog.Debug("Fault reset complete");

            if (run)
            {
                next = Start;
                break;
            }
            next = Idle;
        }
        else
        {
            Arm.faultReset();
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
            eventLog.Info("Stopped normally");
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
                eventLog.Debug("Entered ON state, enter homing");
                next = Home;
            }
            else
            {
                eventLog.Debug("Entered ON state, enter tracking");
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
            eventLog.Debug("Homing complete");
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
            if (!estop || trackingResult)
            {
                eventLog.Warning("Tracking interrupted",
                                 "Tracking encountered error, the robot will be stopped to prevent damage");
            }
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
            eventLog.Warning("Pathing interrupted EStop: " + std::to_string(estop) + " Run: " + std::to_string(run));
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

std::string Robot::FSM::dump() const
{
    std::vector<std::string> lines;
    auto [fx, fy, fz, fr, preResult] = IK::preprocessing(target.x, target.y, target.z, target.r);
    auto [alpha, beta, theta, phi, ikResult] = IK::inverseKinematics(fx, fy, fz, fr);

    lines.push_back(fmt::format("Preprocessing result: {}", IK::resultToString(preResult)));
    lines.push_back(fmt::format("Inverse kinematics result: {}", IK::resultToString(ikResult)));
    lines.push_back(
        fmt::format("Target position:\n\t{:.3}mm {:.3}mm {:.3}mm {:.3}mm", target.x, target.y, target.z, target.r));
    lines.push_back(fmt::format("Proposed position:\n\t{:.3}mm {:.3}mm {:.3}mm {:.3}mm", fx, fy, fz, fr));
    lines.push_back(fmt::format("Proposed joint angles:\n\t{:.3}° {:.3}° {:.3}° {:.3}°", alpha, beta, phi, theta));
    lines.push_back(fmt::format("Actual joint angles:\n\t{:.3}° {:.3}° {:.3}° {:.3}°", J1.getPosition(),
                                J2.getPosition(), J3.getPosition(), J4.getPosition()));
    lines.push_back(fmt::format("Actual velocity:\n\t{:.3}°/s {:.3}°/s {:.3}°/s {:.3}°/s", J1.getVelocity(),
                                J2.getVelocity(), J3.getVelocity(), J4.getVelocity()));
    lines.push_back(fmt::format("Actual torque:\n\t{}% {}% {}% {}%", J1.getTorque(), J2.getTorque(), J3.getTorque(),
                                J4.getTorque()));

    return fmt::format("{}", fmt::join(lines, "\n"));
}
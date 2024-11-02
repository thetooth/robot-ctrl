#include "fsm.hpp"

//! @brief Update the state machine
//!
//! This function updates the state machine of the robot.
//!
//! The state machine has the following states:
//! - Idle: The robot is not doing anything, wait for a command
//!
//! - Reset: Check and report any faults and send a reset command to the drives
//! - Resetting: Check if the reset command has been completed, if not, keep sending the reset command
//!              as they often take multiple cycles to complete
//!
//! - Halt: Disable the drives
//! - Halting: Wait for the drives to enter power off state
//!
//! - Start: Enable the drives
//! - Starting: Wait for the drives to enter power on state
//!
//! - Home: Set mode of operation to homing, set homing offsets and command the drives to home
//! - Homing: Wait for the drives to report homing complete TODO: Add timeout
//!
//! - Track: Set mode of operation to position cyclic
//! - Tracking: Begin tracking the target position with OTG
//!
//! - Jog: Set mode of operation to position cyclic
//! - Jogging: Begin jogging the target position with OTG
void Robot::FSM::update()
{
    // Check if any drives have the emergency stop flag set
    if (Arm.getEmergencyStop() || EtherCATFault)
    {
        if (estop)
        {
            eventLog.Critical("Emergency Stop");
        }
        run = false;
        reset = false;
        estop = false;
    }
    else if (!estop)
    {
        eventLog.Info("Emergency Stop reset");
        estop = true;
    }

    // Update the CoE state machine
    Arm.update();

    // Update counters
    runtimeDuration += CYCLETIME / double(TS::NSEC_PER_SECOND);

    switch (next)
    {
    default:
    case Idle:
        if (reset)
        {
            reset = false;
            next = Reset;
        }

        if (estop && run)
        {
            eventLog.Info("Entering run mode");
            next = Reset;
        }

        break;
    case Reset:
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
            }
        }

        Arm.faultReset();
        next = Resetting;

        break;
    case Resetting: {
        auto pendingErrorCode = false;
        for (auto &&drive : Arm.drives)
        {
            if (drive->getErrorCode() != 0)
            {
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

        inSync = false;
        jog = false;

        next = Halting;
        break;
    case Halting:
        if (J1.compareState(CANOpenState::OFF) && J2.compareState(CANOpenState::OFF))
        {
            eventLog.Info("Stopped normally");
        }
        next = Idle;

        break;
    case Start:
        Arm.setCommand(CANOpenCommand::ENABLE);

        next = Starting;
        break;
    case Starting:
        if (J1.compareState(CANOpenState::ON) && J2.compareState(CANOpenState::ON))
        {
            if (jog)
            {
                eventLog.Debug("Entered ON state, enter jogging");
                next = Jog;
            }
            else if (needsHoming)
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
        configureHoming();

        next = Homing;
    case Homing: {
        auto homingResult = homing();
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
        powerOnDuration += CYCLETIME / TS::NSEC_PER_SECOND;
    }
    break;
    case Jog:
        Arm.setModeOfOperation(CANOpen::control::mode::POSITION_CYCLIC);
        setJoggingDynamics();

        next = Jogging;
        break;
    case Jogging:
        auto joggingResult = jogging();
        if (!estop || !run || joggingResult)
        {
            eventLog.Warning("Jogging interrupted EStop: " + std::to_string(estop) + " Run: " + std::to_string(run));
            inSync = false;
            next = Halt;
            restoreDynamics();
        }
        powerOnDuration += CYCLETIME / double(TS::NSEC_PER_SECOND);
        break;
    }
}

std::string Robot::FSM::to_string() const
{
    switch (next)
    {
    case Idle:
        return "Idle";
    case Reset:
        return "Reset";
    case Resetting:
        return "Resetting";
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
    case Jog:
        return "Jog";
    case Jogging:
        return "Jogging";
    default:
        return "[Unknown State]";
    }
}

//! @brief Dump kinematic and drive information
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
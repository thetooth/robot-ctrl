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
                eventLog.Debug("Fault reset complete");

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
            eventLog.Warning("Tracking interrupted", {
                                                         {"estop", estop},
                                                         {"run", run},
                                                         {"tracking", trackingResult},
                                                     });
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
    lines.push_back("Preprocessing result: " + std::to_string(preResult));
    lines.push_back("Inverse kinematics result: " + std::to_string(ikResult));
    lines.push_back("Target position: " + std::to_string(target.x) + " " + std::to_string(target.y) + " " +
                    std::to_string(target.z) + " " + std::to_string(target.r));
    lines.push_back("Preprocessed position: " + std::to_string(fx) + " " + std::to_string(fy) + " " +
                    std::to_string(fz) + " " + std::to_string(fr));
    lines.push_back("Inverse kinematics position: " + std::to_string(alpha) + " " + std::to_string(beta) + " " +
                    std::to_string(phi) + " " + std::to_string(theta));
    lines.push_back("Current position: " + std::to_string(J1.getPosition()) + " " + std::to_string(J2.getPosition()) +
                    " " + std::to_string(J3.getPosition()) + " " + std::to_string(J4.getPosition()));
    lines.push_back("Current velocity: " + std::to_string(J1.getVelocity()) + " " + std::to_string(J2.getVelocity()) +
                    " " + std::to_string(J3.getVelocity()) + " " + std::to_string(J4.getVelocity()));
    lines.push_back("Current torque: " + std::to_string(J1.getTorque()) + " " + std::to_string(J2.getTorque()) + " " +
                    std::to_string(J3.getTorque()) + " " + std::to_string(J4.getTorque()));

    return fmt::format("{}", fmt::join(lines, "\n"));
}
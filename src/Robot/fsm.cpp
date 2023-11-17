#include "fsm.hpp"

void Robot::FSM::update()
{
    A1.update();
    A2.update();

    switch (next)
    {
    default:
    case Idle:
        if (estop && run)
        {
            diagMsgs.clear();
            diagMsgs.push_back("Entering run mode");

            next = Start;
        }
        if (!estop)
        {
            needsHoming = true;
            next = Halt;
        }
        break;
    case Halt:
        A1.setModeOfOperation(CANOpen::control::mode::NO_MODE);
        A2.setModeOfOperation(CANOpen::control::mode::NO_MODE);
        A1.setCommand(CANOpenCommand::DISABLE);
        A2.setCommand(CANOpenCommand::DISABLE);
        next = Halting;
        break;
    case Halting:
        if (A1.compareState(CANOpenState::OFF) && A2.compareState(CANOpenState::OFF))
        {
            next = Idle;
        }
        break;
    case Start:
        A1.setCommand(CANOpenCommand::ENABLE);
        A2.setCommand(CANOpenCommand::ENABLE);
        next = Starting;
        break;
    case Starting:
        if (A1.compareState(CANOpenState::ON) && A2.compareState(CANOpenState::ON))
        {
            if (needsHoming)
            {
                diagMsgs.push_back("Entered ON state, enter homing");
                next = Home;
            }
            else
            {
                diagMsgs.push_back("Entered ON state, enter tracking");
                next = Track;
            }
        }
        if (!estop || !run)
        {
            next = Halt;
        }
        break;
    case Home:
        A1.setModeOfOperation(CANOpen::control::mode::HOME);
        A2.setModeOfOperation(CANOpen::control::mode::HOME);
        A1.setHomingOffset(-235 * GEAR);
        A2.setHomingOffset(145 * GEAR);
        A1.setCommand(CANOpenCommand::HOME);
        A2.setCommand(CANOpenCommand::HOME);
        next = Homing;
    case Homing: {
        auto homingResult =
            A1.compareState(CANOpenState::HOMING_COMPLETE) && A2.compareState(CANOpenState::HOMING_COMPLETE);
        if (homingResult)
        {
            diagMsgs.push_back("Homing complete");
            needsHoming = false;
            if (trackAfterHoming)
            {
                diagMsgs.push_back("Enter tracking");
                next = Track;
            }
            else
            {
                diagMsgs.push_back("Enter Halt");
                run = false;
                next = Halt;
            }
        }
        if (!estop || !run)
        {
            next = Halt;
        }
        break;
    }
    case Track:
        A1.setModeOfOperation(CANOpen::control::mode::POSITION_CYCLIC);
        A2.setModeOfOperation(CANOpen::control::mode::POSITION_CYCLIC);
        next = Tracking;
        break;
    case Tracking: {
        auto trackingResult = tracking();
        if (!estop || !run || trackingResult)
        {
            diagMsgs.push_back("Tracking interrupted EStop: " + std::to_string(estop) + " Run: " + std::to_string(run) +
                               " Tracking: " + std::to_string(trackingResult));
            inSync = false;
            next = Halt;
        }
        break;
    }
    case Path:
        A1.setModeOfOperation(CANOpen::control::mode::POSITION_CYCLIC);
        A2.setModeOfOperation(CANOpen::control::mode::POSITION_CYCLIC);
        next = Pathing;
        break;
    case Pathing:
        if (!estop || !run)
        {
            diagMsgs.push_back("Pathing interrupted EStop: " + std::to_string(estop) + " Run: " + std::to_string(run));
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

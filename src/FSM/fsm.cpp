#include "fsm.hpp"

void FSM::Robot::update()
{
    //! @brief Update CoE state machine
    //!
    //! NOTE: Trajectory must be computed and sent to the drive PRIOR to entering
    //! CSP, not doing this results in PDO target position being zero at activation
    //! and drive attempting to leave low earth orbit. For safety the target is set
    //! to the current position every cycle.
    A1.update(A1InPDO->status_word);
    A1OutPDO->control_word = A1.getControlWord();
    A1OutPDO->target_position = A1InPDO->actual_position;
    A2.update(A2InPDO->status_word);
    A2OutPDO->control_word = A2.getControlWord();
    A2OutPDO->target_position = A2InPDO->actual_position;

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
        Common::wkc += Common::SetModeOfOperation(A1ID, Common::None);
        Common::wkc += Common::SetModeOfOperation(A2ID, Common::None);
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
        Common::wkc += Common::SetModeOfOperation(A1ID, Common::Homing);
        Common::wkc += Common::SetModeOfOperation(A2ID, Common::Homing);
        Common::wkc += Common::SetHomingOffset(A1ID, -235 * GEAR);
        Common::wkc += Common::SetHomingOffset(A2ID, 145 * GEAR);
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
        Common::wkc += Common::SetModeOfOperation(A1ID, Common::CSP);
        Common::wkc += Common::SetModeOfOperation(A2ID, Common::CSP);
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
    }
}

std::string FSM::Robot::to_string() const
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
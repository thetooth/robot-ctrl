#include "fsm.hpp"

void FSM::Robot::update()
{
    switch (next)
    {
    default:
    case Idle:
        if (estop && run)
        {
            diagMsgs.clear();
            diagMsgs.push_back("Entering run mode");

            A1.setCommand(CANOpenCommand::ENABLE);
            A2.setCommand(CANOpenCommand::ENABLE);
            next = Startup;
        }
        if (!estop)
        {
            needsHoming = true;
            next = Halt;
        }
        break;
    case Halt:
        A1.setCommand(CANOpenCommand::DISABLE);
        A2.setCommand(CANOpenCommand::DISABLE);
        if (A1.compareState(CANOpenState::OFF) && A2.compareState(CANOpenState::OFF))
        {
            Common::wkc += Common::SetModeOfOperation(A1ID, Common::None);
            Common::wkc += Common::SetModeOfOperation(A2ID, Common::None);
            next = Idle;
        }
        break;
    case Startup:
        /* NOTE: Trajectory must be computed and sent to the drive PRIOR
        to entering ON state, not doing this results in PDO target position
        being zero at activation and drive attempting to leave low earth orbit
        */
        if (A1.compareState(CANOpenState::SWITCH_ON) && A2.compareState(CANOpenState::SWITCH_ON))
        {
            A1OutPDO->target_position = A1InPDO->actual_position;
            A2OutPDO->target_position = A2InPDO->actual_position;
        }
        if (A1.compareState(CANOpenState::ON) && A2.compareState(CANOpenState::ON))
        {
            if (needsHoming)
            {
                diagMsgs.push_back("Drives entered ON state, enter homing");
                Common::wkc += Common::SetModeOfOperation(A1ID, Common::Homing);
                Common::wkc += Common::SetModeOfOperation(A2ID, Common::Homing);
                Common::wkc += Common::SetHomingOffset(A1ID, -90 * GEAR);
                next = Homing;
            }
            else
            {
                diagMsgs.push_back("Drives entered ON state, enter tracking");
                Common::wkc += Common::SetModeOfOperation(A1ID, Common::CSP);
                Common::wkc += Common::SetModeOfOperation(A2ID, Common::CSP);
                next = Tracking;
            }
        }
        if (!estop || !run)
        {
            next = Halt;
        }
        break;
    case Homing: {
        auto homingResult = homing();
        if (homingResult)
        {
            diagMsgs.push_back("Homing complete, enter tracking");
            Common::wkc += Common::SetModeOfOperation(A1ID, Common::CSP);
            Common::wkc += Common::SetModeOfOperation(A2ID, Common::CSP);
            needsHoming = false;
            next = Tracking;
        }
        if (!estop || !run)
        {
            next = Halt;
        }
        break;
    }
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

    A1.update(A1InPDO->status_word);
    A1OutPDO->control_word = A1.getControlWord();
    A2.update(A2InPDO->status_word);
    A2OutPDO->control_word = A2.getControlWord();
}
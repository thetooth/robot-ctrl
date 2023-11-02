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
            Common::wkc += Common::Write8(A1ID, 0x6060, 0, 0x8);
            Common::wkc += Common::Write8(A2ID, 0x6060, 0, 0x8);
            A1.setCommand(CANOpenCommand::ENABLE);
            A2.setCommand(CANOpenCommand::ENABLE);
            next = Startup;
        }
        if (!estop)
        {
            next = Halt;
        }
        break;
    case Halt:
        A1.setCommand(CANOpenCommand::DISABLE);
        A2.setCommand(CANOpenCommand::DISABLE);
        if (A1.compareState(CANOpenState::OFF) && A2.compareState(CANOpenState::OFF))
        {
            Common::wkc += Common::Write8(A1ID, 0x6060, 0, 0x0);
            Common::wkc += Common::Write8(A2ID, 0x6060, 0, 0x0);
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
            diagMsgs.push_back("Drives entered switch on state, setting intial target to current target");
            A1OutPDO->target_position = A1InPDO->actual_position;
            A2OutPDO->target_position = A2InPDO->actual_position;
        }
        if (A1.compareState(CANOpenState::ON) && A2.compareState(CANOpenState::ON))
        {
            diagMsgs.push_back("Drives entered on state, enter homing");
            next = Homing;
        }
        break;
    case Homing:
        if (homing())
        {
            diagMsgs.push_back("Homing complete, enter tracking");
            next = Tracking;
        }
        break;
    case Tracking:
        auto res = tracking();
        if (!estop || !run || res)
        {
            diagMsgs.push_back("Tracking interrupted EStop: " + std::to_string(estop) + " Run: " + std::to_string(run) +
                               " Tracking: " + std::to_string(res));
            inSync = false;
            next = Halt;
        }
        break;
    }

    A1.update(A1InPDO->status_word);
    A1OutPDO->control_word = A1.getControlWord();
    A2.update(A2InPDO->status_word);
    A2OutPDO->control_word = A2.getControlWord();
}
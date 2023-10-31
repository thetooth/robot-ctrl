#include "fsm.hpp"

void FSM::update()
{
    switch (next)
    {
    default:
    case Idle:
        if (estop && run)
        {
            next = Startup;
        }
        if (!estop)
        {
            next = Halt;
        }
        break;
    case Halt:
        next = Idle;
        break;
    case Startup:
        /* NOTE: Trajectory must be computed and sent to the drive PRIOR
        to entering ON state, not doing this results in PDO target position
        being zero at activation and drive attempting to leave low earth orbit
        */
        if (A1.getState() == CANOpenState::SWITCH_ON && A2.getState() == CANOpenState::SWITCH_ON)
        {
            next = Homing;
        }
        break;
    case Homing:
        next = Tracking;
        break;
    case Tracking:
        if (!estop || !run)
        {
            next = Halt;
        }
        break;
    }

    A1.update(A1InPDO->status_word);
    A1OutPDO->control_word = A1.getControlWord();
    A2.update(A2InPDO->status_word);
    A2OutPDO->control_word = A2.getControlWord();
}
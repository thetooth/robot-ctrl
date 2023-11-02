#include "fsm.hpp"

/*
    Operation steps:
        1. Set OD 6060h to 06h to set the mode as Homing mode.
        2. Set OD 607Ch for the home offset.
        3. Set OD 6098h for the homing method.
        4. Set OD 6099h sub1 for the speed when searching for the home switch.
        5. Set OD 6099h sub2 for the speed when searching for the Z pulse.
        6. Set OD 609Ah for the homing acceleration.
    Read the servo drive information:
        1. Read OD 6041h to obtain the servo drive status.
        2. Read OD 6064h to obtain the actual value of the motor position at present.
*/

bool FSM::Robot::homing()
{
    A1.setCommand(CANOpenCommand::HOME);
    A2.setCommand(CANOpenCommand::HOME);
    // spdlog::debug("{0:x} {0:x}", A1InPDO->status_word, A2InPDO->status_word);
    return A1.compareState(CANOpenState::HOMING_COMPLETE) && A2.compareState(CANOpenState::HOMING_COMPLETE);
}
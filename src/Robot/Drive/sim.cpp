#include "sim.hpp"
#include "CAN/CoE.hpp"

uint16_t Sim::PDO::getStatusWord() const
{
    return status_word;
}

int32_t Sim::PDO::getActualPosition() const
{
    return target_position;
}

int32_t Sim::PDO::getActualVelocity() const
{
    return simulated_velocity;
}

int16_t Sim::PDO::getActualTorque() const
{
    return target_torque;
}

int32_t Sim::PDO::getFollowingError() const
{
    return 0;
}

uint16_t Sim::PDO::getErrorCode() const
{
    return 0;
}

uint32_t Sim::PDO::getDigitalInputs() const
{
    return 0;
}

bool Sim::PDO::getEmergencyStop() const
{
    return false;
}

void Sim::PDO::setControlWord(uint16_t value)
{
    control_word = value;
    stepSimulation();
}

void Sim::PDO::setTargetPosition(int32_t value)
{
    target_position = value;
    stepSimulation();
}

void Sim::PDO::stepSimulation()
{
    if (control_word == CANOpen::control::word::FAULT_RESET)
    {
        status_word = CANOpen::status::value::READY_TO_SWITCH_ON_STATE;
    }
    else if (control_word == CANOpen::control::word::SWITCH_ON_OR_DISABLE_OPERATION)
    {
        status_word = CANOpen::status::value::READY_TO_SWITCH_ON_STATE;
    }
    else if (control_word == CANOpen::control::word::ENABLE_OPERATION)
    {
        status_word = CANOpen::status::value::ON_STATE;
    }
    else if (control_word == CANOpen::control::word::SET_ABS_POINT_NOBLEND)
    {
        status_word = CANOpen::status::value::HOMING_COMPLETE_STATE;
    }
    else
    {
        status_word = CANOpen::status::value::OFF_STATE;
    }

    if (status_word & CANOpen::status::value::ON_STATE)
    {
        simulated_velocity = (target_position - previous_position) * 1000;
        previous_position = target_position;
    }
    else
    {
        simulated_velocity = 0;
    }
}
#include "settings.hpp"
#include "fsm.hpp"

void Robot::to_json(json &j, const OTGSettings &s)
{
    j = json{{"max-velocity", s.max_velocity}, {"max-acceleration", s.max_acceleration}, {"max-jerk", s.max_jerk}};
}

void Robot::from_json(const json &j, OTGSettings &s)
{
    s.max_velocity = j.value("max-velocity", 100.0);
    s.max_acceleration = j.value("max-acceleration", 100.0);
    s.max_jerk = j.value("max-jerk", 100.0);
}

void Robot::FSM::updateDynamics(std::vector<Robot::OTGSettings> settings)
{
    if (run)
    {
        spdlog::warn("Not updating dynamics because we're moving");
        return;
    }

    if (settings.size() < input.degrees_of_freedom)
    {
        spdlog::warn(
            "Not enough objects in updateDynamics payload for number of configured DoF which is {} and I have {}",
            input.degrees_of_freedom, settings.size());
        return;
    }
    for (size_t i = 0; i <= input.degrees_of_freedom - 1; i++)
    {
        input.max_velocity[i] = settings.at(i).max_velocity;
        input.max_acceleration[i] = settings.at(i).max_acceleration;
        input.max_jerk[i] = settings.at(i).max_jerk;
    }
}

void Robot::FSM::setJoggingDynamics()
{
    if (next == Jogging || next == Tracking)
    {
        return;
    }

    for (size_t i = 0; i <= input.degrees_of_freedom - 1; i++)
    {
        previousDynamics[i] = {input.max_velocity[i], input.max_acceleration[i], input.max_jerk[i]};
    }

    input.max_velocity = {100.0, 100.0, 10000.0, 10000.0};
    input.max_acceleration = {100.0, 100.0, 10000.0, 10000.0};
    input.max_jerk = {100.0, 100.0, 10000.0, 10000.0};
}

void Robot::FSM::restoreDynamics()
{
    for (size_t i = 0; i <= input.degrees_of_freedom - 1; i++)
    {
        input.max_velocity[i] = previousDynamics[i].max_velocity;
        input.max_acceleration[i] = previousDynamics[i].max_acceleration;
        input.max_jerk[i] = previousDynamics[i].max_jerk;
    }
}
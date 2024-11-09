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

void Robot::to_json(json &j, const Preset &p)
{
    j = json{{"id", p.id},
             {"name", p.name},
             {"axisConfigurations", p.axisConfigurations},
             {"synchronisationMethod", p.synchronisationMethod}};
}

void Robot::from_json(const json &j, Preset &p)
{
    j.at("id").get_to(p.id);
    j.at("name").get_to(p.name);
    j.at("axisConfigurations").get_to(p.axisConfigurations);
    j.at("synchronisationMethod").get_to(p.synchronisationMethod);
}

void Robot::FSM::updateDynamics(Robot::Preset settings)
{
    if (run)
    {
        spdlog::warn("Not updating dynamics because we're moving");
        return;
    }

    if (settings.axisConfigurations.size() < input.degrees_of_freedom)
    {
        spdlog::warn(
            "Not enough objects in updateDynamics payload for number of configured DoF which is {} and I have {}",
            input.degrees_of_freedom, settings.axisConfigurations.size());
        return;
    }
    for (size_t i = 0; i <= input.degrees_of_freedom - 1; i++)
    {
        input.max_velocity[i] = settings.axisConfigurations.at(i).max_velocity;
        input.max_acceleration[i] = settings.axisConfigurations.at(i).max_acceleration;
        input.max_jerk[i] = settings.axisConfigurations.at(i).max_jerk;
    }

    static std::unordered_map<std::string, ruckig::Synchronization> const SynchronisationMethodTable = {
        {"none", ruckig::Synchronization::None},
        {"time", ruckig::Synchronization::Time},
        {"timeIfNecessary", ruckig::Synchronization::TimeIfNecessary},
        {"phase", ruckig::Synchronization::Phase}};

    auto syncMethod = SynchronisationMethodTable.find(settings.synchronisationMethod);
    if (syncMethod != SynchronisationMethodTable.end())
    {
        input.synchronization = syncMethod->second;
    }
    else
    {
        spdlog::warn("Unknown synchronisation method: {}", settings.synchronisationMethod);
    }
}

void Robot::FSM::setJoggingDynamics()
{
    if (next == State::Jogging || next == State::Tracking)
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
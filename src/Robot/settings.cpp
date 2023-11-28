#include "settings.hpp"
#include "fsm.hpp"

void Robot::to_json(json &j, const OTGSettings &s)
{
    j = json{{"max-velocity", s.max_velocity}, {"max-acceleration", s.max_acceleration}, {"max-jerk", s.max_jerk}};
}

void Robot::from_json(const json &j, OTGSettings &s)
{
    s.max_velocity = j.value("max-velocity", 600.0);
    s.max_acceleration = j.value("max-acceleration", 50000.0);
    s.max_jerk = j.value("max-jerk", 600.0);
}

void Robot::FSM::receiveSettings([[maybe_unused]] natsConnection *nc, [[maybe_unused]] natsSubscription *sub,
                                 natsMsg *msg, [[maybe_unused]] void *closure)
{
    auto payload = json::parse(natsMsg_GetData(msg));
    natsMsg_Destroy(msg);

    if (!payload.is_array())
    {
        spdlog::warn("Got something that isn't an array in receiveSettings");
        return;
    }

    std::vector<Robot::OTGSettings> settings;
    for (auto &&axis : payload)
    {
        settings.push_back(axis.template get<Robot::OTGSettings>());
    }

    // Do not apply if we're moving to avoid helicoptering
    if (!run)
    {
        if (settings.size() < input.degrees_of_freedom)
        {
            spdlog::warn(
                "Not enough objects in receiveSettings payload for number of configured DoF which is {} and I have {}",
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
}

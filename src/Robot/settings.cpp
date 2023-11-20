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
    auto settings = payload.template get<Robot::OTGSettings>();

    // Do not apply if we're moving to avoid helicoptering
    if (!run)
    {
        input.max_velocity = {settings.max_velocity, settings.max_velocity};
        input.max_acceleration = {settings.max_acceleration, settings.max_acceleration};
        input.max_jerk = {settings.max_jerk, settings.max_jerk};
    }

    natsMsg_Destroy(msg);
}
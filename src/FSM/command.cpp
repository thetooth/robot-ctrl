#include "fsm.hpp"

void FSM::Robot::commandCb([[maybe_unused]] natsConnection *nc, [[maybe_unused]] natsSubscription *sub, natsMsg *msg,
                           [[maybe_unused]] void *closur)
{
    auto payload = json::parse(natsMsg_GetData(msg));
    if (!payload["command"].is_string())
    {
        spdlog::warn("Invalid JSON received in command handler");
        return;
    }
    auto command = payload["command"].template get<std::string>();

    if (command.compare("start") == 0 && estop)
    {
        run = true;
    }
    if (command.compare("goto") == 0 && estop)
    {
        if (!payload["position"].is_object())
        {
            spdlog::warn("Invalid JSON received in goto handler");
            return;
        }
        dx = payload["position"]["x"].template get<double>();
        dy = payload["position"]["y"].template get<double>();
    }
    if (command.compare("stop") == 0)
    {
        run = false;
    }
    natsMsg_Destroy(msg);
}
#include "fsm.hpp"

void FSM::Robot::commandCb([[maybe_unused]] natsConnection *nc, [[maybe_unused]] natsSubscription *sub, natsMsg *msg,
                           [[maybe_unused]] void *closur)
{
    try
    {
        auto payload = json::parse(natsMsg_GetData(msg));
        auto command = payload["command"].template get<std::string>();

        if (command.compare("start") == 0 && estop)
        {
            run = true;
        }
        if (command.compare("goto") == 0 && estop)
        {
            auto x = payload["position"]["x"].template get<double>();
            auto y = payload["position"]["y"].template get<double>();

            // Commit only if parsing both values succeeds
            dx = x, dy = y;
        }
        if (command.compare("reset") == 0)
        {
            needsHoming = true;
        }
        if (command.compare("stop") == 0)
        {
            run = false;
        }
    }
    catch (const json::parse_error &e)
    {
        spdlog::error("commandCb parsing error: {}", e.what());
    }
    catch (const json::exception &e)
    {
        spdlog::error("commandCb exception: {}", e.what());
    }
    natsMsg_Destroy(msg);
}
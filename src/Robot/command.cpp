#include "fsm.hpp"

void Robot::FSM::receiveCommand([[maybe_unused]] natsConnection *nc, [[maybe_unused]] natsSubscription *sub,
                                natsMsg *msg, [[maybe_unused]] void *closure)
{
    try
    {
        auto payload = json::parse(natsMsg_GetData(msg));
        auto command = payload["command"].template get<std::string>();

        if (command.compare("stop") == 0)
        {
            run = false;
        }
        if (command.compare("start") == 0 && estop)
        {
            run = true;
        }
        if (command.compare("goto") == 0 && estop)
        {
            target = payload["pose"].template get<IK::Pose>();
        }
        if (command.compare("waypoints") == 0 && estop)
        {
            if (payload["waypoints"].is_array())
            {
                waypoints.clear();
                for (auto &&waypoint : payload["waypoints"])
                {
                    waypoints.push_back(waypoint.template get<IK::Pose>());
                }
            }
        }
        if (command.compare("reset") == 0)
        {
            reset = true;
            if (!run)
            {
                next = Robot::Idle;
            }
        }
        if (command.compare("home") == 0)
        {
            needsHoming = true;
            run = true;
        }
        if (command.compare("hotStart") == 0)
        {
            needsHoming = false;
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
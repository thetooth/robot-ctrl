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
            jog = false;
        }
        if (command.compare("start") == 0 && estop)
        {
            run = true;
        }
        if (command.compare("goto") == 0 && estop && !jog)
        {
            target = payload["pose"].template get<IK::Pose>();
        }
        if (command.compare("jog") == 0 && estop)
        {

            run = true;
            jog = true;
            auto jog = payload["jog"].template get<IK::Pose>();
            // Jogging is relative to the current position of the actual joints
            target.alpha = J1.getPosition() + jog.alpha;
            target.beta = J2.getPosition() + jog.beta;
            target.theta = J3.getPosition() + jog.theta;
            target.phi = J4.getPosition() + jog.phi;
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
        if (command.compare("setHome") == 0)
        {
            auto pose = payload["pose"].template get<IK::Pose>();
            J1.setHomingOffset(pose.alpha);
            J2.setHomingOffset(pose.beta);
            J3.setHomingOffset(pose.theta);
            J4.setHomingOffset(pose.phi);

            J1.setHomingMode(35);
            J2.setHomingMode(35);
            J3.setHomingMode(35);
            J4.setHomingMode(35);

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
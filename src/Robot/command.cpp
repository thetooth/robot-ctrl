#include "fsm.hpp"

void Robot::FSM::receiveCommand(json payload)
{

    auto command = payload["command"].template get<std::string>();

    static std::unordered_map<std::string, Command> commandMap = {
        {"stop", Command::Stop},
        {"start", Command::Start},
        {"goto", Command::Goto},
        {"jog", Command::Jog},
        {"waypoints", Command::Waypoints},
        {"reset", Command::Reset},
        {"home", Command::Home},
        {"setHome", Command::SetHome},
        {"hotStart", Command::HotStart},
        {"moveLinear", Command::MoveLinear},
        {"moveCircular", Command::MoveCircular},
    };

    auto cmd = commandMap.find(command);
    if (cmd == commandMap.end())
    {
        spdlog::error("Unknown command: {}", command);
        return;
    }

    switch (cmd->second)
    {
    case Command::Stop:
        run = false;
        jog = false;
        break;
    case Command::Start:
        if (estop)
        {
            run = true;
        }
        break;
    case Command::Goto:
        if (estop && !jog)
        {
            target = payload["pose"].template get<IK::Pose>();
        }
        break;
    case Command::MoveLinear:
        if (estop && !jog)
        {
            run = true;
            auto end = payload["pose"].template get<IK::Pose>();
            auto duration = payload["duration"].template get<double>();
            auto steps = duration * CYCLETIME / 1000;

            // Get the last waypoint or the target position as the origin
            auto start = waypoints.empty() ? target : waypoints.back();
            auto [path, result] = Motion::linearInterpolation(start, end, steps);
            if (result != IK::Result::Success)
            {
                KinematicAlarm = true;
                eventLog.Kinematic(fmt::format("MoveLinear failed: {}", IK::resultToString(result)));
                return;
            }
            waypoints.insert(waypoints.end(), path.begin(), path.end());
        }
        break;
    case Command::Jog:
        if (estop)
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
        break;
    case Command::Waypoints:
        if (estop)
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
        break;
    case Command::Reset:
        reset = true;
        if (!run)
        {
            next = FSM::State::Idle;
        }
        break;
    case Command::Home:
        needsHoming = true;
        run = true;
        break;
    case Command::SetHome: {
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
    break;
    case Command::HotStart:
        needsHoming = false;
        break;
    default:
        break;
    }
}
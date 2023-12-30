#include "fsm.hpp"

void Robot::to_json(json &j, const OTGStatus &p)
{
    j = json{{"result", p.result}};
}

void Robot::to_json(json &j, const EtherCATStatus &p)
{
    j = json{{"interval", p.interval}, {"drift", p.drift}, {"integral", p.integral}};
}

void Robot::to_json(json &j, const Status &p)
{
    j = json{
        {"run", p.run},       {"alarm", p.alarm},     {"state", p.state}, {"otg", p.otg},
        {"ethercat", p.ethercat}, {"drives", p.drives}, {"diagMsg", p.diagMsg}, {"pose", p.pose},
    };
}

void Robot::FSM::broadcastStatus(natsConnection *nc)
{
    if (nc == nullptr)
    {
        spdlog::critical("broadcastStatus called with nullptr");
        return;
    }

    auto alarm = false;
    if (J1.fault || J2.fault || KinematicAlarm)
    {
        alarm = true;
    }

    auto diagStr = std::string("");
    for (auto msg : diagMsgs)
    {
        diagStr.append(msg + "\n");
    }

    auto [dx, dy, dz, dr] =
        IK::forwardKinematics(J1.getPosition(), J2.getPosition(), input.current_position[2], input.current_position[3]);

    status.run = run;
    status.alarm = alarm;
    status.state = to_string();
    status.diagMsg = diagStr;
    status.pose = IK::Pose{
        .x = dx,
        .y = dy,
        .z = dz,
        .r = dr,
        .alpha = J1.getPosition(),
        .beta = J2.getPosition(),
        .phi = input.current_position[2],
        .theta = input.current_position[3],
        .alphaVelocity = J1.getVelocity(),
        .betaVelocity = J2.getVelocity(),
        .phiVelocity = input.current_velocity[2],
        .thetaVelocity = input.current_velocity[3],
    };

    json j = status;
    auto payload = j.dump();

    auto ncStatus = natsConnection_Publish(nc, "motion.status", payload.c_str(), payload.length());
    if (ncStatus != NATS_OK)
    {
        spdlog::error("Failed to broadcast status: {}", natsStatus_GetText(ncStatus));
    }
}

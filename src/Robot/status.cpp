#include "fsm.hpp"

void Robot::to_json(json &j, const OTGStatus &p)
{
    j = json{{"result", p.result}};
}

void Robot::to_json(json &j, const Status &p)
{
    j = json{
        {"run", p.run}, {"alarm", p.alarm},     {"state", p.state},
        {"otg", p.otg}, {"diagMsg", p.diagMsg}, {"pose", p.pose},
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
    if (A1Fault || A2Fault || KinematicAlarm)
    {
        alarm = true;
    }

    auto diagStr = std::string("");
    for (auto msg : diagMsgs)
    {
        diagStr.append(msg + "\n");
    }

    auto [dx, dy] = IK::forwardKinematics(A1.getPosition(), A2.getPosition());

    status.run = run;
    status.alarm = alarm;
    status.state = to_string();
    status.diagMsg = diagStr;
    status.pose = IK::Pose{
        .x = dx,
        .y = dy,
        .z = input.current_position[3],
        .r = target.r,
        .alpha = A1.getPosition(),
        .beta = A2.getPosition(),
        .phi = input.current_position[2],
        .alphaVelocity = A1.getVelocity(),
        .betaVelocity = A2.getVelocity(),
        .phiVelocity = input.current_velocity[2],
    };

    json j = status;
    auto payload = j.dump();

    auto ncStatus = natsConnection_Publish(nc, "motion.status", payload.c_str(), payload.length());
    if (ncStatus != NATS_OK)
    {
        spdlog::error("Failed to broadcast status: {}", natsStatus_GetText(ncStatus));
    }
}
#include "fsm.hpp"

void Robot::to_json(json &j, const OTGStatus &p)
{
    j = json{{"result", p.result}, {"kinematicResult", p.kinematicResult}};
}

void Robot::to_json(json &j, const EtherCATStatus &p)
{
    j = json{{"interval", p.interval},
             {"sync0", p.sync0},
             {"compensation", p.compensation},
             {"integral", p.integral},
             {"state", p.state}};
}

void Robot::to_json(json &j, const MotorStatus &p)
{
    j = json{
        {"slaveID", p.slaveID},
        {"statusWord", p.statusWord},
        {"controlWord", p.controlWord},
        {"errorCode", p.errorCode},
        {"fault", p.fault},
        {"lastFault", p.lastFault},
        {"actualTorque", p.actualTorque},
        {"followingError", p.followingError},
    };
}

void Robot::to_json(json &j, const Status &p)
{
    j = json{
        {"run", p.run},
        {"estop", p.estop},
        {"alarm", p.alarm},
        {"needsHoming", p.needsHoming},
        {"state", p.state},
        {"otg", p.otg},
        {"ethercat", p.ethercat},
        {"drives", p.drives},
        {"diagMsg", p.diagMsg},
        {"pose", p.pose},
        {"runtimeDuration", p.runtimeDuration},
        {"powerOnDuration", p.powerOnDuration},
        {"cpuTemperature", p.cpuTemperature},
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
    if (J1.fault || J2.fault || KinematicAlarm || EtherCATFault || !estop)
    {
        alarm = true;
    }

    for (auto event : eventLog)
    {
        json j = event;
        auto payload = j.dump();
        auto ncStatus = natsConnection_Publish(nc, "motion.event", payload.c_str(), payload.length());
        if (ncStatus != NATS_OK)
        {
            spdlog::error("Failed to broadcast event: {}", natsStatus_GetText(ncStatus));
            continue;
        }
        eventLog.pop_front();
    }

    auto [dx, dy, dz, dr] = IK::forwardKinematics(J1.getPosition(), J2.getPosition(), J3.getPosition(),
                                                  J4.getPosition(), target.toolOffset);

    status.run = run;
    status.estop = estop;
    status.alarm = alarm;
    status.state = to_string();
    status.needsHoming = needsHoming;
    // status.diagMsg = diagStr;
    status.pose = IK::Pose{
        .x = dx,
        .y = dy,
        .z = dz,
        .r = dr,
        .alpha = J1.getPosition(),
        .beta = J2.getPosition(),
        .theta = J3.getPosition(),
        .phi = J4.getPosition(),
        .alphaVelocity = J1.getVelocity(),
        .betaVelocity = J2.getVelocity(),
        .thetaVelocity = J3.getVelocity(),
        .phiVelocity = J4.getVelocity(),
    };
    status.runtimeDuration = runtimeDuration;
    status.powerOnDuration = powerOnDuration;

    status.drives.clear();
    auto drives = {&J1, &J2, &J3, &J4};
    for (auto drive : drives)
    {
        status.drives.push_back({
            .slaveID = drive->slaveID,
            .statusWord = drive->pdo->getStatusWord(),
            .controlWord = drive->getControlWord(),
            .errorCode = drive->pdo->getErrorCode(),
            .fault = drive->fault,
            .lastFault = drive->lastFault,
            .actualTorque = drive->getTorque(),
            .followingError = drive->getFollowingError(),
        });
    }

    json j = status;
    auto payload = j.dump();

    auto ncStatus = natsConnection_Publish(nc, "motion.status", payload.c_str(), payload.length());
    if (ncStatus != NATS_OK)
    {
        spdlog::error("Failed to broadcast status: {}", natsStatus_GetText(ncStatus));
    }
}

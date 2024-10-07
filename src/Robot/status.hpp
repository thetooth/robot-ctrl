#ifndef ROBOT_STATUS_HPP
#define ROBOT_STATUS_HPP

#include "nlohmann/json.hpp"
#include "ruckig/ruckig.hpp"

namespace Robot
{
    using namespace ruckig;
    using json = nlohmann::json;

    struct OTGStatus
    {
        ruckig::Result result;
        IK::Result kinematicResult;
    };
    void to_json(json &j, const OTGStatus &p);

    struct EtherCATStatus
    {
        int64_t interval;
        int64_t sync0;
        int64_t compensation;
        int64_t integral;
        int64_t state;
    };
    void to_json(json &j, const EtherCATStatus &p);

    struct MotorStatus
    {
        int slaveID;
        uint16_t statusWord;
        uint16_t controlWord;
        uint16_t errorCode;
        bool fault;
        std::string lastFault = "OK";
        double actualTorque;
        double followingError;
    };
    void to_json(json &j, const MotorStatus &p);

    struct Status
    {
        bool run;
        bool estop;
        bool alarm;
        bool needsHoming;
        std::string state;
        OTGStatus otg;
        EtherCATStatus ethercat;
        std::vector<MotorStatus> drives;
        std::string diagMsg;
        IK::Pose pose;
        double runtimeDuration;
        double powerOnDuration;
    };
    void to_json(json &j, const Status &p);
} // namespace Robot

#endif

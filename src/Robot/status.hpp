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
    };
    void to_json(json &j, const OTGStatus &p);

    struct Status
    {
        bool run;
        bool alarm;
        std::string state;
        OTGStatus otg;
        std::string diagMsg;
        IK::Pose pose;
    };
    void to_json(json &j, const Status &p);
} // namespace Robot

#endif
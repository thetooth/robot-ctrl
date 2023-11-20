#ifndef ROBOT_SETTINGS_HPP
#define ROBOT_SETTINGS_HPP

#include "nlohmann/json.hpp"
#include "ruckig/ruckig.hpp"

namespace Robot
{
    using namespace ruckig;
    using json = nlohmann::json;

    struct OTGSettings
    {
        double max_velocity;
        double max_acceleration;
        double max_jerk;
    };
    void to_json(json &j, const OTGSettings &s);
    void from_json(const json &j, OTGSettings &s);
} // namespace Robot

#endif
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

    struct DynamicsItem
    {
        std::string name;
        std::array<OTGSettings, 4> axisConfigurations;
    };

    struct DynamicsStore
    {
        uint selected = 0;
        std::vector<DynamicsItem> items;
    };
} // namespace Robot

#endif
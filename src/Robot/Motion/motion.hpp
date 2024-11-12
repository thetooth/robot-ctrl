#pragma once

#include <deque>
#include <tuple>

#include "../IK/scara.hpp"
#include "nlohmann/json.hpp"
#include "ruckig/ruckig.hpp"
#include "spdlog/spdlog.h"

namespace Motion
{
    using json = nlohmann::json;
    using namespace ruckig;

    std::tuple<std::deque<IK::Pose>, IK::Result> linearInterpolation(const IK::Pose &start, const IK::Pose &end,
                                                                     double stepSize);
    std::deque<IK::Pose> circularInterpolation(const IK::Pose &start, double radius, double stepSize);

    std::tuple<std::deque<IK::Pose>, ruckig::Result> calculateIntermediatePath(const ruckig::InputParameter<4> input,
                                                                               std::vector<IK::Pose> &waypoints);
    std::tuple<std::array<double, 4>, std::array<double, 4>, std::array<double, 4>, std::array<double, 4>,
               ruckig::Result>
    calculateMaximal(ruckig::InputParameter<4> input);
} // namespace Motion

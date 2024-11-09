#pragma once

#include <deque>
#include <tuple>

#include "../IK/scara.hpp"
#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"

namespace Motion
{
    using json = nlohmann::json;

    std::tuple<std::deque<IK::Pose>, IK::Result> linearInterpolation(const IK::Pose &start, const IK::Pose &end,
                                                                     double stepSize);
    std::deque<IK::Pose> circularInterpolation(const IK::Pose &start, double radius, double stepSize);
} // namespace Motion

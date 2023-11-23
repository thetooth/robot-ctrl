#ifndef IK_SCARA_HPP
#define IK_SCARA_HPP

#include <math.h>
#include <stdio.h>
#include <tuple>

#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"

namespace IK
{
    const auto L1 = 200.0;
    const auto L2 = 200.0;

    std::tuple<double, double> forwardKinematics(double theta1, double theta2);
    std::tuple<double, double, double, bool> inverseKinematics(double x, double y);
    std::tuple<double, double, bool> preprocessing(double x, double y);

    using json = nlohmann::json;
    struct Pose
    {
        double x, y;
        double alpha, beta;
        double alphaVelocity, betaVelocity;
    };
    void to_json(json &j, const Pose &p);
    void from_json(const json &j, Pose &p);
} // namespace IK

#endif
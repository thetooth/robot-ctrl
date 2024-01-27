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
    const auto ScrewPitch = 16.0 / 360;

    enum Result
    {
        Success,
        JointLimit,
        Singularity,
    };
    std::string resultToString(Result result);

    std::tuple<double, double, double, double> forwardKinematics(double alpha, double beta, double theta, double phi);
    std::tuple<double, double, double, double, Result> inverseKinematics(double x, double y, double z, double r);
    std::tuple<double, double, double, double, Result> preprocessing(double x, double y, double z, double r);

    using json = nlohmann::json;
    struct Pose
    {
        double x, y, z, r;
        double alpha, beta, theta, phi;
        double alphaVelocity, betaVelocity;
        double thetaVelocity, phiVelocity;
    };
    void to_json(json &j, const Pose &p);
    void from_json(const json &j, Pose &p);
} // namespace IK

#endif
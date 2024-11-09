#ifndef IK_SCARA_HPP
#define IK_SCARA_HPP

#include <deque>
#include <math.h>
#include <stdio.h>
#include <tuple>

#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"

namespace IK
{
    const auto L1 = 200.0;               // Length of the first link
    const auto L2 = 200.0;               // Length of the second link
    const auto ScrewPitch = 16.0 / 360;  // Screw pitch of the z axis ball screw
    const auto AlphaMin = -45.0;         // Minimum angle of the first joint
    const auto AlphaMax = 225.0;         // Maximum angle of the first joint
    const auto BetaMin = -150.0;         // Minimum angle of the second joint
    const auto BetaMax = 150.0;          // Maximum angle of the second joint
    const auto BaseKeepOut = 100.0;      // Keep out distance from the base
    const auto BaseKeepOutBorder = 10.0; // Keep out distance from the base buffer

    using json = nlohmann::json;
    struct Pose
    {
        double x, y, z, r;
        double alpha, beta, theta, phi;
        double toolOffset;
        double alphaVelocity, betaVelocity;
        double thetaVelocity, phiVelocity;
    };
    void to_json(json &j, const Pose &p);
    void from_json(const json &j, Pose &p);

    enum class Result
    {
        Success,
        JointLimit,
        Singularity,
        ForwardKinematic,
    };
    std::string resultToString(Result result);

    std::tuple<double, double, double, double> forwardKinematics(double alpha, double beta, double theta, double phi,
                                                                 double toolOffset = 0);
    std::tuple<double, double, double, double, Result> inverseKinematics(double x, double y, double z, double r,
                                                                         double toolOffset = 0);
    std::tuple<double, double, double, double, Result> preprocessing(double x, double y, double z, double r);
    std::tuple<double, double, double, double, Result> postprocessing(double alpha, double beta, double theta,
                                                                      double phi);

} // namespace IK

#endif
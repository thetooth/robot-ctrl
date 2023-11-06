#ifndef IK_SCARA_HPP
#define IK_SCARA_HPP

#include <math.h>
#include <stdio.h>
#include <tuple>

#include "spdlog/spdlog.h"

namespace IKScara
{
    const auto L1 = 200.0;
    const auto L2 = 200.0;

    std::tuple<double, double> forwardKinematics(double theta1, double theta2);
    std::tuple<double, double, double, bool> inverseKinematics(double x, double y);
    std::tuple<double, double, bool> preprocessing(double x, double y);
    double gap(double target, double current, bool *alarm);
} // namespace IKScara

#endif
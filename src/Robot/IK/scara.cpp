#include "scara.hpp"

std::tuple<double, double> IK::forwardKinematics(double alpha, double beta)
{
    double alphaF = alpha * M_PI / 180; // degrees to radians
    double betaF = beta * M_PI / 180;
    auto xP = L1 * cos(alphaF) + L2 * cos(alphaF + betaF);
    auto yP = L1 * sin(alphaF) + L2 * sin(alphaF + betaF);

    return {xP, yP};
}

std::tuple<double, double, double, bool> IK::inverseKinematics(double x, double y)
{
    double alpha, beta, phi = 0;

    beta = acos((pow(x, 2.0) + pow(y, 2.0) - pow(L1, 2.0) - pow(L2, 2.0)) / (2 * L1 * L2));
    if (x < 0 & y < 0)
    {
        beta = (-1) * beta;
    }

    alpha = atan(x / y) - atan((L2 * sin(beta)) / (L1 + L2 * cos(beta)));

    beta = (-1) * beta * 180 / M_PI;
    alpha = alpha * 180 / M_PI;

    // Somethings not right...
    if (isnan(alpha) || isnan(beta))
    {
        return {alpha, beta, phi, false};
    }

    const auto maxAngle = 150.0;

    // Angles adjustment depending in which quadrant the final tool coordinate x,y is
    if (x >= 0 && y >= 0)
    { // 1st quadrant
        alpha = 90 - alpha;
    }
    if (x < 0 && y > 0)
    { // 2nd quadrant
        alpha = 90 - alpha;
    }
    if (x < 0 && y < 0)
    { // 3rd quadrant
        alpha = 270 - alpha;
        beta = std::min(maxAngle, beta); // Prevent elbow under collision with A1 (radius)
        phi = 270 - alpha - beta;
        phi = (-1) * phi;
    }
    if (x >= 0 && y < 0)
    { // 4th quadrant
        alpha = -90 - alpha;
    }
    if (x < 0 && y == 0)
    {
        alpha = 90 - alpha;
    }

    // Prevent elbow under collision with base during transition between 2nd and 3rd quadrant
    alpha = std::min(270 - 30.0, alpha);
    // Prevent elbow over collision with A1 (radius)
    beta = std::max(-maxAngle, beta);

    // Calculate "phi" angle so gripper is parallel to the X axis
    phi = 90 + alpha + beta;
    phi = (-1) * phi;

    // Angle adjustment depending in which quadrant the final tool coordinate x,y is
    if (x < 0 && y < 0)
    { // 3d quadrant
        phi = 270 - alpha - beta;
    }
    if (abs(phi) > 165)
    {
        phi = 180 + phi;
    }

    return {alpha, beta, phi, true};
}

std::tuple<double, double, bool> IK::preprocessing(double x, double y)
{
    const auto baseKeepOut = 100.0;
    auto ok = true;

    // Prevent placing the tool directly behind the base
    if (x < 0 && y <= 0)
    { // 3rd quadrant
        x = std::min(x, -baseKeepOut);
        ok = x < -baseKeepOut;
    }
    if (x >= 0 && y <= 0)
    { // 4th quadrant
        x = std::max(x, baseKeepOut);
        ok = x > baseKeepOut;
    }

    return {x, y, ok};
}

void IK::to_json(json &j, const Pose &p)
{
    j = json{{"x", p.x}, {"y", p.x}, {"alpha", p.alpha}, {"beta", p.beta}};
}
void IK::from_json(const json &j, Pose &p)
{
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
    try
    {
        j.at("alpha").get_to(p.alpha);
        j.at("beta").get_to(p.beta);
    }
    catch (const json::exception &e)
    {
        spdlog::trace("Decoding IK::Pose: {}", e.what());
    }
}
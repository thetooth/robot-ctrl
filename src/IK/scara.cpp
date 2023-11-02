#include "scara.hpp"

std::tuple<double, double> IKScara::forwardKinematics(double theta1, double theta2)
{
    double theta1F = theta1 * M_PI / 180; // degrees to radians
    double theta2F = theta2 * M_PI / 180;
    auto xP = L1 * cos(theta1F) + L2 * cos(theta1F + theta2F);
    auto yP = L1 * sin(theta1F) + L2 * sin(theta1F + theta2F);

    return {xP, yP};
}

std::tuple<double, double, double> IKScara::inverseKinematics(double x, double y)
{
    double theta1, theta2, phi = 0;

    theta2 = acos((pow(x, 2.0) + pow(y, 2.0) - pow(L1, 2.0) - pow(L2, 2.0)) / (2 * L1 * L2));
    if (x < 0 & y < 0)
    {
        theta2 = (-1) * theta2;
    }

    theta1 = atan(x / y) - atan((L2 * sin(theta2)) / (L1 + L2 * cos(theta2)));

    theta2 = (-1) * theta2 * 180 / M_PI;
    theta1 = theta1 * 180 / M_PI;

    // Angles adjustment depending in which quadrant the final tool coordinate x,y is
    if (x >= 0 && y >= 0)
    { // 1st quadrant
        theta1 = 90 - theta1;
    }
    if (x < 0 && y > 0)
    { // 2nd quadrant
        theta1 = 90 - theta1;
    }
    if (x < 0 && y < 0)
    { // 3d quadrant
        theta1 = 270 - theta1;
        phi = 270 - theta1 - theta2;
        phi = (-1) * phi;
    }
    if (x > 0 && y < 0)
    { // 4th quadrant
        theta1 = -90 - theta1;
    }
    if (x < 0 && y == 0)
    {
        theta1 = 270 + theta1;
    }

    // Calculate "phi" angle so gripper is parallel to the X axis
    phi = 90 + theta1 + theta2;
    phi = (-1) * phi;

    // Angle adjustment depending in which quadrant the final tool coordinate x,y is
    if (x < 0 && y < 0)
    { // 3d quadrant
        phi = 270 - theta1 - theta2;
    }
    if (abs(phi) > 165)
    {
        phi = 180 + phi;
    }

    return {theta1, theta2, phi};
}

double IKScara::gap(double target, double current, bool *alarm)
{
    *alarm = false;
    if (abs(target - current) > 15)
    {
        *alarm = true;
        spdlog::warn("GAP: {}", abs(target - current));
        return current;
    }

    return target;
}
#include "scara.hpp"

// Forward kinematics
// Returns x, y, z, r in cartesian coordinates
std::tuple<double, double, double, double> IK::forwardKinematics(double alpha, double beta, double theta, double phi)
{
    double alphaF = alpha * M_PI / 180; // degrees to radians
    double betaF = beta * M_PI / 180;
    double phiF = -90 + alpha + beta;
    auto xP = L1 * cos(alphaF) + L2 * cos(alphaF + betaF);
    auto yP = L1 * sin(alphaF) + L2 * sin(alphaF + betaF);
    auto zP = (theta - phi) * ScrewPitch;
    auto rP = (-1) * (phiF + phi);

    return {xP, yP, zP, rP};
}

// Inverse kinematics
// Returns alpha, beta, phi, theta in joint space
std::tuple<double, double, double, double, IK::Result> IK::inverseKinematics(double x, double y, double z, double r)
{
    IK::Result result = IK::Result::Success;
    double alpha, beta, phi, theta = 0;

    beta = acos((pow(x, 2.0) + pow(y, 2.0) - pow(L1, 2.0) - pow(L2, 2.0)) / (2 * L1 * L2));
    if (x < 0 && y < 0)
    {
        beta = (-1) * beta;
    }

    alpha = atan(x / y) - atan((L2 * sin(beta)) / (L1 + L2 * cos(beta)));

    beta = (-1) * beta * 180 / M_PI;
    alpha = alpha * 180 / M_PI;

    // Somethings not right...
    if (isnan(alpha) || isnan(beta))
    {
        return {alpha, beta, phi, theta, IK::Result::Singularity};
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
        if (beta > maxAngle)
        {
            result = IK::Result::JointLimit;
        }
        beta = std::min(maxAngle, beta); // Prevent elbow under collision with J1 (radius)
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
    if (alpha > 270 - 30.0)
    {
        result = IK::Result::JointLimit;
    }
    alpha = std::min(270 - 30.0, alpha);
    // Prevent elbow over collision with J1 (radius)
    if (beta < -maxAngle)
    {
        result = IK::Result::JointLimit;
    }
    beta = std::max(-maxAngle, beta);

    // Calculate "phi" angle so gripper is parallel to the X axis
    phi = -90 + alpha + beta + r;
    phi = (-1) * phi;

    // Calculate "theta" angle so that the gripper is at the correct height during rotation
    theta = phi + z / ScrewPitch;

    return {alpha, beta, theta, phi, result};
}

std::tuple<double, double, double, double, IK::Result> IK::preprocessing(double x, double y, double z, double r)
{
    const auto baseKeepOut = 100.0;
    IK::Result result = IK::Result::Success;

    // Prevent placing the tool directly behind the base
    if (x < 0 && y <= 0)
    { // 3rd quadrant
        if (x > -baseKeepOut)
        {
            result = IK::Result::JointLimit;
        }
        x = std::min(x, -baseKeepOut);
    }
    if (x >= 0 && y <= 0)
    { // 4th quadrant
        if (x < baseKeepOut)
        {
            result = IK::Result::JointLimit;
        }
        x = std::max(x, baseKeepOut);
    }
    z = std::max(z, 0.0);
    r = std::min(std::max(r, -180.0), 180.0);

    return {x, y, z, r, result};
}

void IK::to_json(json &j, const Pose &p)
{
    j = json{
        {"x", p.x},
        {"y", p.y},
        {"z", p.z},
        {"r", p.r},
        {"alpha", p.alpha},
        {"beta", p.beta},
        {"phi", p.phi},
        {"theta", p.theta},
        {"alphaVelocity", p.alphaVelocity},
        {"betaVelocity", p.betaVelocity},
        {"phiVelocity", p.phiVelocity},
        {"thetaVelocity", p.thetaVelocity},
    };
}
void IK::from_json(const json &j, Pose &p)
{
    p.x = j.value("x", 0.0);
    p.y = j.value("y", 0.0);
    p.z = j.value("z", 0.0);
    p.r = j.value("r", 0.0);

    p.alpha = j.value("alpha", 0.0);
    p.beta = j.value("beta", 0.0);
    p.phi = j.value("phi", 0.0);
    p.theta = j.value("theta", 0.0);
}

std::string IK::resultToString(IK::Result result)
{
    switch (result)
    {
    case IK::Result::Success:
        return "Success";
    case IK::Result::JointLimit:
        return "Joint Limit";
    case IK::Result::Singularity:
        return "Singularity";
    default:
        return "Unknown";
    }
}
#include "scara.hpp"

// Forward kinematics
// Returns x, y, z, r in cartesian coordinates
std::tuple<double, double, double, double> IK::forwardKinematics(double alpha, double beta, double theta, double phi,
                                                                 double toolOffset)
{
    double alphaF = alpha * M_PI / 180; // degrees to radians
    double betaF = beta * M_PI / 180;
    double phiF = alpha + beta;
    auto xP = L1 * cos(alphaF) + L2 * cos(alphaF + betaF);
    auto yP = L1 * sin(alphaF) + L2 * sin(alphaF + betaF);
    auto zP = (theta - phi) * ScrewPitch;
    auto rP = (-1) * (phiF + phi);

    xP += toolOffset * sin(rP * M_PI / 180);
    yP += toolOffset * cos(rP * M_PI / 180);

    return {xP, yP, zP, rP};
}

// Inverse kinematics
// Returns alpha, beta, phi, theta in joint space
std::tuple<double, double, double, double, IK::Result> IK::inverseKinematics(double x, double y, double z, double r,
                                                                             double toolOffset)
{
    IK::Result result = IK::Result::Success;
    double alpha, beta, phi, theta = 0;

    // Tool tip offset
    x = x - toolOffset * sin(r * M_PI / 180);
    y = y - toolOffset * cos(r * M_PI / 180);

    // Cosine of the tool from the origin using the law of cosines
    double c2 = (pow(x, 2) + pow(y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2);
    // Sine of the tool from the origin
    double s2 = sqrt(1 - pow(c2, 2.0));
    // Effective length along the x axis
    double k1 = L1 + L2 * c2;
    // Effective length along the y axis
    double k2 = L2 * s2;

    // Calculate the alpha angle
    // atan2(x, y) is the angle between the tool and origin
    // atan2(k1, k2) accounts for the effective angle of the second link
    alpha = atan2(k1, k2) - atan2(y, x);
    // Calculate the beta angle
    // atan2(s2, c2) is the angle between the first and second link
    beta = atan2(s2, c2);

    // Inversion of the elbow when in 3rd quadrant
    if (x < 0 && y < 0)
    {
        alpha = (-1) * (atan2(k1, k2) - atan2(y, -x));
        beta = (-1) * beta;
    }

    beta = (-1) * beta * 180 / M_PI;
    alpha = 90 - (alpha * 180 / M_PI);

    // Somethings not right...
    if (isnan(alpha) || isnan(beta))
    {
        return {alpha, beta, phi, theta, IK::Result::Singularity};
    }

    // Prevent tool collision with J1
    if (beta < BetaMin || beta > BetaMax)
    {
        result = IK::Result::JointLimit;
    }
    beta = std::max(BetaMin, std::min(BetaMax, beta));

    // Prevent J1 from colliding with the base
    if (alpha < AlphaMin || alpha > AlphaMax)
    {
        result = IK::Result::JointLimit;
    }
    alpha = std::max(AlphaMin, std::min(AlphaMax, alpha));

    // Calculate "phi" angle so gripper is parallel to the X axis
    phi = alpha + beta + r;
    phi = (-1) * phi;

    // Calculate "theta" angle so that the gripper is at the correct height during rotation
    theta = phi + z / ScrewPitch;

    return {alpha, beta, theta, phi, result};
}

std::tuple<double, double, double, double, IK::Result> IK::preprocessing(double x, double y, double z, double r)
{
    IK::Result result = IK::Result::Success;

    // Prevent placing the tool directly behind the base
    if (x < 0 && y <= 0)
    { // 3rd quadrant
        if (x > -(BaseKeepOut + BaseKeepOutBorder))
        {
            result = IK::Result::JointLimit;
        }
        x = std::min(x, -(BaseKeepOut + BaseKeepOutBorder));
    }
    if (x >= 0 && y <= 0)
    { // 4th quadrant
        if (x < BaseKeepOut + BaseKeepOutBorder)
        {
            result = IK::Result::JointLimit;
        }
        x = std::max(x, BaseKeepOut + BaseKeepOutBorder);
    }
    z = std::max(z, 0.0);
    r = std::min(std::max(r, -180.0), 180.0);

    return {x, y, z, r, result};
}

std::tuple<double, double, double, double, IK::Result> IK::postprocessing(double alpha, double beta, double theta,
                                                                          double phi)
{
    IK::Result result = IK::Result::Success;

    // Perform forward kinematics to check on the tool position
    auto [fx, fy, fz, fr] = forwardKinematics(alpha, beta, theta, phi);

    // Prevent the tool from colliding with the base
    if (fx < 0 && fy <= 0)
    { // 3rd quadrant
        if (fx > -BaseKeepOut)
        {
            result = IK::Result::ForwardKinematic;
        }
    }
    if (fx >= 0 && fy <= 0)
    { // 4th quadrant
        if (fx < BaseKeepOut)
        {
            result = IK::Result::ForwardKinematic;
        }
    }

    return {alpha, beta, theta, phi, result};
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
        {"toolOffset", p.toolOffset},
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

    p.toolOffset = j.value("toolOffset", 0.0);
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
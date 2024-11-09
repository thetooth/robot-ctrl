#include "motion.hpp"

std::tuple<std::deque<IK::Pose>, IK::Result> Motion::linearInterpolation(const IK::Pose &start, const IK::Pose &end,
                                                                         double steps)
{
    std::deque<IK::Pose> path;

    for (int i = 0; i < steps; i++)
    {
        auto dt = i / steps;
        auto x = start.x + (end.x - start.x) * dt;
        auto y = start.y + (end.y - start.y) * dt;
        auto z = start.z + (end.z - start.z) * dt;
        auto r = start.r + (end.r - start.r) * dt;
        auto toolOffset = start.toolOffset + (end.toolOffset - start.toolOffset) * dt;

        auto [alpha, beta, theta, phi, result] = IK::inverseKinematics(x, y, z, r, toolOffset);
        if (result != IK::Result::Success)
        {
            spdlog::warn("Failed to interpolate: {}", resultToString(result));
            return {path, result};
        }

        path.push_back({.x = x,
                        .y = y,
                        .z = z,
                        .r = r,
                        .alpha = alpha,
                        .beta = beta,
                        .theta = theta,
                        .phi = phi,
                        .toolOffset = toolOffset});
    }

    return {path, IK::Result::Success};
}
#include "motion.hpp"

//! @brief Intermediate waypoint path calculation
//!
//! This function calculates the intermediate path between the given waypoints.
//! Implementation based on https://arxiv.org/pdf/2407.13423v1
//!
//! Once the maximum acceleration of a point to point trajectory is known, we then
//! need to determine the max allowable entry acceleration for the next point.
//!
//! This is done by performing a binary search such that
//! max_acceleration = min(output1.max_acceleration, input2.max_acceleration)
//!
//! @param origin The initial input parameters for the Ruckig OTG
//! @param waypoints The waypoints to calculate the path between
//! @return A tuple containing the intermediate path and the result of the calculation
std::tuple<std::deque<IK::Pose>, ruckig::Result> Motion::calculateIntermediatePath(
    const ruckig::InputParameter<4> origin, std::vector<IK::Pose> &waypoints)
{
    std::deque<IK::Pose> path;
    std::deque<std::array<double, 4>> max_acceleration;
    std::deque<std::array<double, 4>> max_velocity;

    ruckig::InputParameter<4> input;
    ruckig::InputParameter<4> input2;
    ruckig::OutputParameter<4> output;
    ruckig::Result result;

    // Set the initial input parameters
    input.current_position = origin.current_position;
    input.current_velocity = origin.current_velocity;
    input.current_acceleration = origin.current_acceleration;
    input.max_velocity = origin.max_velocity;
    input.max_acceleration = origin.max_acceleration;
    input.max_jerk = origin.max_jerk;

    input2.max_velocity = origin.max_velocity;
    input2.max_acceleration = origin.max_acceleration;
    input2.max_jerk = origin.max_jerk;

    // Find max acceleration for each point to point trajectory
    for (auto waypoint = waypoints.begin(); waypoint != waypoints.end(); ++waypoint)
    {
        if (waypoint == waypoints.end())
        {
            max_acceleration.push_back({0.0, 0.0, 0.0, 0.0});
            max_velocity.push_back({0.0, 0.0, 0.0, 0.0});
            break;
        }

        // Set current position
        if (waypoint != waypoints.begin())
        {
            input.current_position = jointVector(*(waypoint - 1));
        }

        // Set the target position
        input.target_position = jointVector(*waypoint);

        // Get the maximal acceleration and velocity for this waypoint
        auto [aMax, vMax, aMin, vMin, result] = calculateMaximal(input);
        if (result != ruckig::Result::Finished)
        {
            return {path, result};
        }

        // If there is another waypoint determine the min of the max acceleration
        double mag = 1.0;
        size_t iterMax = 1000;
        for (size_t iter = 1; iter <= iterMax; iter++)
        {
            input2.current_position = jointVector(*waypoint);
            for (size_t i = 0; i < 4; i++)
            {
                // input2.current_acceleration[i] = aMax[i] * mag;
                input2.current_velocity[i] = vMax[i] * mag;
            }

            input2.target_position = jointVector(*(waypoint + 1));
            input2.target_acceleration = {0.0, 0.0, 0.0, 0.0};
            input2.target_velocity = {0.0, 0.0, 0.0, 0.0};
            // input2.target_acceleration = aMax;
            // input2.target_velocity = vMax;

            auto [aMax2, vMax2, aMin2, vMin2, result] = calculateMaximal(input2);
            if (result != ruckig::Result::Finished)
            {
                return {path, result};
            }

            int goodAxis = 0;
            for (size_t i = 0; i < 4; i++)
            {
                // if (i == 0)
                // {
                //     spdlog::debug("Axis: {}, Target: {}, Current: {}, AMax: {}, AMin: {} VMax: {}, VMin: {}", i,
                //                   input.target_position[i], input.current_position[i], aMax2[i], aMin2[i], vMax2[i],
                //                   vMin2[i]);
                // }
                if (input.target_position[i] > input.current_position[i] && vMin2[i] >= 0)
                {
                    goodAxis++;
                }
                else if (input.target_position[i] < input.current_position[i] && vMin2[i] <= 0)
                {
                    goodAxis++;
                }
                else if (input.target_position[i] == input.current_position[i] && vMin2[i] == 0)
                {
                    goodAxis++;
                }
            }

            if (goodAxis == 4 || iter == iterMax)
            {
                spdlog::info("Entry acceleration has solution at iteration {}", iter);

                for (size_t i = 0; i < 4; i++)
                {
                    // spdlog::debug("Axis: {}, Target: {}, Current: {}, AMax: {}, AMin: {} AMax2: {}, AMin2: {}", i,
                    //               input.target_position[i], input.current_position[i], aMax[i], aMin[i], aMax2[i],
                    //               aMin2[i]);
                    if (input.target_position[i] < input.current_position[i])
                    {
                        aMax[i] = std::min(aMax[i], aMax2[i]);
                        vMax[i] = std::min(vMax[i], vMax2[i]);
                    }
                    else
                    {
                        aMax[i] = std::max(aMax[i], aMax2[i]);
                        vMax[i] = std::max(vMax[i], vMax2[i]);
                    }
                }
                spdlog::debug("Maximal Acceleration: [{}, {}, {}, {}]", aMax[0], aMax[1], aMax[2], aMax[3]);
                break;
            }
            else
            {
                mag = mag * 0.5; // Binary search
            }
        }

        max_acceleration.push_back(aMax);
        max_velocity.push_back(vMax);

        // spdlog::debug("Maximal Acceleration: [{}, {}, {}, {}]", aMax[0], aMax[1], aMax[2], aMax[3]);
        // spdlog::debug("Maximal Velocity: [{}, {}, {}, {}]", vMax[0], vMax[1], vMax[2], vMax[3]);
    }

    // Generate the path
    Ruckig<4> otg{1e6 / double(1e+9)};
    for (auto waypoint = waypoints.begin(); waypoint != waypoints.end(); ++waypoint)
    {
        otg.reset();
        // std::vector<IK::Pose>::iterator previous = waypoint - 1;
        auto index = std::distance(waypoints.begin(), waypoint);

        input.target_position = jointVector(*waypoint);
        // input.target_acceleration = max_acceleration[index];
        input.target_velocity = max_velocity[index];

        if (waypoint == waypoints.begin())
        {
            input.current_position = origin.current_position;
            input.current_acceleration = origin.current_acceleration;
            input.current_velocity = origin.current_velocity;
        }

        if (waypoint != waypoints.begin())
        {
            input.current_acceleration = {0.0, 0.0, 0.0, 0.0};
            input.current_velocity = {0.0, 0.0, 0.0, 0.0};
        }

        result = ruckig::Result::Working;
        while (result == ruckig::Result::Working)
        {
            result = otg.update(input, output);
            if (result != ruckig::Result::Working)
            {
                break;
            }

            auto [x, y, z, r] = IK::forwardKinematics(output.new_position[0], output.new_position[1],
                                                      output.new_position[2], output.new_position[3]);

            IK::Pose pose = {
                .x = x,
                .y = y,
                .z = z,
                .r = r,
            };

            // spdlog::info("Pose: [{}, {}, {}, {}]", pose.x, pose.y, pose.z, pose.r);

            output.pass_to_input(input);

            path.push_back(pose);
        }
    }

    return {path, ruckig::Result::Finished};
}

//! @brief Calculate the maximal acceleration and velocity for a given path
//!
//! This function calculates the maximal acceleration and velocity for a given path.
//!
//! @param input The input parameters for the Ruckig OTG
//! @return A tuple containing the maximal acceleration, maximal velocity, and the result of the calculation
std::tuple<std::array<double, 4>, std::array<double, 4>, std::array<double, 4>, std::array<double, 4>, ruckig::Result>
Motion::calculateMaximal(ruckig::InputParameter<4> input)
{
    // Create the Ruckig OTG instance
    Ruckig<4> otg{1e6 / double(1e+9)};
    ruckig::OutputParameter<4> output;
    ruckig::Result result;

    otg.reset();

    // Update the OTG
    result = ruckig::Result::Working;
    std::array<double, 4> maxA = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> maxV = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> minA = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> minV = {0.0, 0.0, 0.0, 0.0};

    while (result == ruckig::Result::Working)
    {
        result = otg.update(input, output);
        if (result != ruckig::Result::Working)
        {
            break;
        }

        for (size_t i = 0; i < 4; i++)
        {
            if (input.target_position[i] > input.current_position[i])
            {
                maxA[i] = std::max(output.new_acceleration[i], maxA[i]);
                maxV[i] = std::max(output.new_velocity[i], maxV[i]);
                minA[i] = std::min(output.new_acceleration[i], minA[i]);
                minV[i] = std::min(output.new_velocity[i], minV[i]);
            }
            else
            {
                maxA[i] = std::min(output.new_acceleration[i], maxA[i]);
                maxV[i] = std::min(output.new_velocity[i], maxV[i]);
                minA[i] = std::max(output.new_acceleration[i], minA[i]);
                minV[i] = std::max(output.new_velocity[i], minV[i]);
            }
        }

        output.pass_to_input(input);
    }

    return {maxA, maxV, minA, minV, result};
}

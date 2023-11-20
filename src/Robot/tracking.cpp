#include "fsm.hpp"

bool Robot::FSM::tracking()
{
    if (!inSync)
    {
        // Set input parameters
        input.current_position = {
            A1.getPosition(),
            A2.getPosition(),
        };
        input.current_velocity = {0.0, 0.0};
        input.current_acceleration = {0.0, 0.0};
        otg.reset();

        spdlog::debug("Resync");
        diagMsgs.push_back("Resync OTG to actual position");
        inSync = true;
    }

    auto [fx, fy, preOk] = IK::preprocessing(target.x, target.y);
    auto [alpha, beta, phi, ikOk] = IK::inverseKinematics(fx, fy);
    if (ikOk)
    {
        input.target_position[0] = alpha;
        input.target_position[1] = beta;
    }
    KinematicAlarm = !preOk || !ikOk || alpha == 240 || beta == -150;

    status.otg.result = otg.update(input, output);
    auto &p = output.new_position;

    A1Fault = A1.move(p[0]);
    A2Fault = A2.move(p[1]);

    if (A1Fault || A2Fault)
    {
        diagMsgs.push_back("Drive fault occurred, stopping");
        diagMsgs.push_back("A1 " + A1.lastFault);
        diagMsgs.push_back("A2 " + A2.lastFault);
        run = false;
        return true;
    }

    // Set input parameters
    output.pass_to_input(input);
    // spdlog::trace("position: {:03.2f} {:03.2f} {:03.2f} {:03.2f}", alpha, beta, p[0], p[1]);
    // spdlog::debug("deg/s {} {}", A1.getVelocity(), A2.getVelocity());

    return false;
}
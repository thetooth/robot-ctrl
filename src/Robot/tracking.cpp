#include "fsm.hpp"

bool Robot::FSM::tracking()
{
    if (!inSync)
    {
        // Set input parameters
        input.current_position = {
            J1.getPosition(),
            J2.getPosition(),
        };
        input.current_velocity = {
            J1.getVelocity(),
            J2.getVelocity(),
        };
        input.current_acceleration = {0.0, 0.0};
        otg.reset();

        spdlog::debug("Resync");
        diagMsgs.push_back("Resync OTG to actual position");
        inSync = true;
    }

    auto [fx, fy, fz, fr, preOk] = IK::preprocessing(target.x, target.y, target.z, target.r);
    auto [alpha, beta, phi, theta, ikOk] = IK::inverseKinematics(fx, fy, fz, fr);
    if (ikOk)
    {
        input.target_position[0] = alpha;
        input.target_position[1] = beta;
        input.target_position[2] = phi;
        input.target_position[3] = theta;
    }
    KinematicAlarm = !preOk || !ikOk || alpha == 240 || beta == -150;

    status.otg.result = otg.update(input, output);
    auto &p = output.new_position;

    if (J1.move(p[0]) || J2.move(p[1]))
    {
        diagMsgs.push_back("Drive fault occurred, stopping");
        diagMsgs.push_back("J1 " + J1.lastFault);
        diagMsgs.push_back("J2 " + J2.lastFault);
        run = false;
        return true;
    }

    // Set input parameters
    output.pass_to_input(input);
    // spdlog::trace("position: {:03.2f} {:03.2f} {:03.2f} {:03.2f}", alpha, beta, p[0], p[1]);
    // spdlog::debug("deg/s {} {}", J1.getVelocity(), J2.getVelocity());

    return false;
}
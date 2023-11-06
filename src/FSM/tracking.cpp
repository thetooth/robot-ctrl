#include "fsm.hpp"

bool FSM::Robot::tracking()
{
    if (!inSync)
    {
        // Set input parameters
        input.current_position = {
            A1InPDO->actual_position / GEAR,
            A2InPDO->actual_position / GEAR,
        };
        input.current_velocity = {0.0, 0.0};
        input.current_acceleration = {0.0, 0.0};
        otg.reset();

        spdlog::debug("Resync");
        diagMsgs.push_back("Resync OTG to actual position");
        inSync = true;
    }

    auto [fx, fy, preOk] = IKScara::preprocessing(dx, dy);
    auto [alpha, beta, phi, ikOk] = IKScara::inverseKinematics(fx, fy);
    if (ikOk)
    {
        input.target_position[0] = alpha;
        input.target_position[1] = beta;
    }
    KinematicAlarm = !preOk || !ikOk;

    auto res = otg.update(input, output);
    if (res != ruckig::Finished)
    {
    }
    auto &p = output.new_position;

    A1OutPDO->target_position = IKScara::gap(p[0], A1InPDO->actual_position / GEAR, &A1GapAlarm) * GEAR;
    A2OutPDO->target_position = IKScara::gap(p[1], A2InPDO->actual_position / GEAR, &A2GapAlarm) * GEAR;

    if (A1GapAlarm || A2GapAlarm)
    {
        diagMsgs.push_back("Gap between current and target position exceeds dynamic capabilities");
        diagMsgs.push_back("A1 target " + std::to_string(p[0]) + ", actual " +
                           std::to_string(A1InPDO->actual_position / GEAR));
        diagMsgs.push_back("A2 target " + std::to_string(p[1]) + ", actual " +
                           std::to_string(A2InPDO->actual_position / GEAR));
        run = false;
        return true;
    }

    // Set input parameters
    output.pass_to_input(input);
    spdlog::trace("position: {:03.2f} {:03.2f} {:03.2f} {:03.2f}", alpha, beta, p[0], p[1]);

    return false;
}
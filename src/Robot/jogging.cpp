#include "fsm.hpp"

bool Robot::FSM::jogging()
{
    if (!inSync)
    {
        // Set input parameters
        input.current_position = {
            J1.getPosition(),
            J2.getPosition(),
            J3.getPosition(),
            J4.getPosition(),
        };
        input.current_velocity = {
            J1.getVelocity(),
            J2.getVelocity(),
            J3.getVelocity(),
            J4.getVelocity(),
        };
        input.current_acceleration = {0.0, 0.0};
        otg.reset();

        eventLog.Kinematic("Resync OTG to actual position");
        inSync = true;
    }

    input.target_position[0] = target.alpha;
    input.target_position[1] = target.beta;
    input.target_position[2] = target.theta;
    input.target_position[3] = target.phi;

    status.otg.result = otg.update(input, output);
    auto &p = output.new_position;

    if (J1.move(p[0]) || J2.move(p[1]) || J3.move(p[2]) || J4.move(p[3]))
    {
        for (auto &&drive : Arm.drives)
        {
            if (drive->fault)
            {
                eventLog.Error("J" + std::to_string(drive->slaveID) + " " + drive->lastFault, dump());
            }
        }
        jog = false;
        run = false;
        return true;
    }

    // Set input parameters
    output.pass_to_input(input);

    return false;
}
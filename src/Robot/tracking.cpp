#include "fsm.hpp"

bool Robot::FSM::tracking()
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
        input.current_acceleration = {0.0, 0.0, 0.0, 0.0};
        otg.reset();

        eventLog.Kinematic("Resync OTG to actual position");
        inSync = true;
    }

    auto [fx, fy, fz, fr, preResult] = IK::preprocessing(target.x, target.y, target.z, target.r);
    status.otg.kinematicResult = preResult;
    if (preResult == IK::Result::JointLimit && !KinematicAlarm)
    {
        eventLog.Kinematic("Joint limit exceeded during preprocessing", dump());
    }

    auto [alpha, beta, theta, phi, ikResult] = IK::inverseKinematics(fx, fy, fz, fr, target.toolOffset);
    status.otg.kinematicResult = (preResult != IK::Result::Success ? preResult : ikResult);

    if (ikResult != IK::Result::Singularity)
    {
        input.target_position[0] = alpha;
        input.target_position[1] = beta;
        input.target_position[2] = theta;
        input.target_position[3] = phi;
    }
    if (ikResult == IK::Result::JointLimit && !KinematicAlarm)
    {
        eventLog.Kinematic("Joint limit exceeded during kinematic step", dump());
    }
    if (ikResult == IK::Result::Singularity && !KinematicAlarm)
    {
        eventLog.Kinematic("Singularity detected", dump());
    }
    KinematicAlarm = preResult != IK::Result::Success || ikResult != IK::Result::Success;

    status.otg.result = otg.update(input, output);
    auto &p = output.new_position;

    auto [d1, d2, d3, d4, postResult] = IK::postprocessing(p[0], p[1], p[2], p[3]);
    if (postResult == IK::Result::ForwardKinematic)
    {
        KinematicAlarm = true;
        eventLog.Error("Forward kinematic test detected imminent crash", dump());
        run = false;
        return true;
    }

    if (J1.move(d1) || J2.move(d2) || J3.move(d3) || J4.move(d4))
    {
        for (auto &&drive : Arm.drives)
        {
            if (drive->fault)
            {
                eventLog.Error("J" + std::to_string(drive->slaveID) + " " + drive->lastFault, dump());
            }
        }
        run = false;
        return true;
    }

    // Set input parameters
    output.pass_to_input(input);

    return false;
}
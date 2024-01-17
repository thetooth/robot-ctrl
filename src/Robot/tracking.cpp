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
        input.current_acceleration = {0.0, 0.0};
        otg.reset();

        eventLog.Info("Resync OTG to actual position");
        inSync = true;
    }

    auto [fx, fy, fz, fr, preResult] = IK::preprocessing(target.x, target.y, target.z, target.r);
    status.otg.kinematicResult = preResult;

    auto [alpha, beta, theta, phi, ikResult] = IK::inverseKinematics(fx, fy, fz, fr);
    status.otg.kinematicResult = (preResult != IK::Result::Success ? preResult : ikResult);

    KinematicAlarm = preResult != IK::Result::Success || ikResult != IK::Result::Success;
    if (ikResult != IK::Result::Singularity)
    {
        input.target_position[0] = alpha;
        input.target_position[1] = beta;
        input.target_position[2] = theta;
        input.target_position[3] = phi;
    }
    // else
    // {
    //     eventLog.push_back("Kinematic alarm");
    //     eventLog.push_back("Preprocessing result: " + std::to_string(preResult));
    //     eventLog.push_back("Inverse kinematics result: " + std::to_string(ikResult));
    //     eventLog.push_back("Target position: " + std::to_string(target.x) + " " + std::to_string(target.y) + " " +
    //                        std::to_string(target.z) + " " + std::to_string(target.r));
    //     eventLog.push_back("Preprocessed position: " + std::to_string(fx) + " " + std::to_string(fy) + " " +
    //                        std::to_string(fz) + " " + std::to_string(fr));
    //     eventLog.push_back("Inverse kinematics position: " + std::to_string(alpha) + " " + std::to_string(beta) + " "
    //     +
    //                        std::to_string(phi) + " " + std::to_string(theta));
    //     eventLog.push_back("Current position: " + std::to_string(J1.getPosition()) + " " +
    //                        std::to_string(J2.getPosition()) + " " + std::to_string(J3.getPosition()) + " " +
    //                        std::to_string(J4.getPosition()));
    //     eventLog.push_back("Current velocity: " + std::to_string(J1.getVelocity()) + " " +
    //                        std::to_string(J2.getVelocity()) + " " + std::to_string(J3.getVelocity()) + " " +
    //                        std::to_string(J4.getVelocity()));
    //     eventLog.push_back("Current torque: " + std::to_string(J1.getTorque()) + " " + std::to_string(J2.getTorque())
    //     +
    //                        " " + std::to_string(J3.getTorque()) + " " + std::to_string(J4.getTorque()));
    // }

    status.otg.result = otg.update(input, output);
    auto &p = output.new_position;

    if (J1.move(p[0]) || J2.move(p[1]) || J3.move(p[2]) || J4.move(p[3]))
    {
        eventLog.Error("Drive fault occurred, stopping");
        for (auto &&drive : Arm.drives)
        {
            if (drive->fault)
            {
                eventLog.Error("J" + std::to_string(drive->slaveID) + ": " + drive->lastFault);
            }
        }
        run = false;
        return true;
    }

    // Set input parameters
    output.pass_to_input(input);
    // spdlog::trace("position: {:03.2f} {:03.2f} {:03.2f} {:03.2f}", alpha, beta, p[0], p[1]);
    // spdlog::debug("deg/s {} {}", J1.getVelocity(), J2.getVelocity());

    return false;
}
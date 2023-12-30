#ifndef ROBOT_FSM_HPP
#define ROBOT_FSM_HPP

#include <deque>

#include "ethercat.h"
#include "nats.h"
#include "nlohmann/json.hpp"
#include "ruckig/ruckig.hpp"

#include "../common.hpp"
#include "Drive/group.hpp"
#include "IK/scara.hpp"
#include "settings.hpp"
#include "status.hpp"

namespace Robot
{
    using namespace ruckig;
    using json = nlohmann::json;

    enum State
    {
        Idle,
        Halt,
        Halting,
        Start,
        Starting,
        Home,
        Homing,
        Track,
        Tracking,
        Path,
        Pathing
    };

    class FSM
    {
      public:
        bool run;
        bool estop = true;
        bool needsHoming = false;
        bool trackAfterHoming = true;
        bool inSync;

        State next = Idle;
        std::deque<std::string> diagMsgs = {};

        Drive::Motor J1;
        Drive::Motor J2;
        Drive::Motor J3;
        Drive::Motor J4;
        Drive::Group Arm;

        // Create instances: the Ruckig OTG as well as input and output parameters
        Ruckig<4> otg{CYCLETIME / double(TS::NSEC_PER_SECOND)}; // control cycle
        InputParameter<4> input;
        OutputParameter<4> output;

        // Target
        IK::Pose target = {
            .x = 0,
            .y = 150,
            .z = 0,
            .r = 0,
            .alpha = 0,
            .beta = 0,
            .phi = 0,
            .theta = 0,
            .alphaVelocity = 0,
            .betaVelocity = 0,
            .phiVelocity = 0,
            .thetaVelocity = 0,
        };
        // Waypoints
        std::vector<IK::Pose> waypoints;

        Status status;

        bool KinematicAlarm = false;

        FSM()
        {
            // Set dynamic limits
            input.max_velocity = {300.0, 300.0, 40000.0, 40000.0};
            input.max_acceleration = {2500.0, 2500.0, 400000.0, 400000.0};
            input.max_jerk = {10000.0, 10000.0, 1000000.0, 1000000.0};

            // Set initial conditions
            input.current_position = {0.0, 0.0, 0.0, 0.0};
            input.current_velocity = {0.0, 0.0, 0.0, 0.0};
            input.current_acceleration = {0.0, 0.0, 0.0, 0.0};
            input.target_position = {0.0, 0.0, 0.0, 0.0};
            input.target_velocity = {0.0, 0.0, 0.0, 0.0};
            input.synchronization = Synchronization::Phase;
        }

        void update();
        bool tracking();
        void receiveCommand(natsConnection *nc, natsSubscription *sub, natsMsg *msg, void *closure);
        void receiveSettings(natsConnection *nc, natsSubscription *sub, natsMsg *msg, void *closure);
        void broadcastStatus(natsConnection *nc = nullptr);
        std::string to_string() const;
    };
} // namespace Robot
#endif

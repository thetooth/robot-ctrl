#ifndef FSM_ROBOT_HPP
#define FSM_ROBOT_HPP

#include <deque>

#include "ethercat.h"
#include "nats.h"
#include "nlohmann/json.hpp"
#include "ruckig/ruckig.hpp"

#include "../CAN/CoEFSM.h"
#include "../IK/scara.hpp"
#include "../common.hpp"
#include "../delta.hpp"

namespace FSM
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
    };

    class Robot
    {
      public:
        bool run;
        bool estop = true;
        bool needsHoming = true;
        bool trackAfterHoming = true;
        bool inSync;

        State next = Idle;
        std::deque<std::string> diagMsgs = {};

        CoEFSM A1;
        CoEFSM A2;

        // Create instances: the Ruckig OTG as well as input and output parameters
        Ruckig<2> otg{0.002}; // control cycle
        InputParameter<2> input;
        OutputParameter<2> output;

        double dx, dy = 150;

        // PDO memory
        Delta::tx_t *A1InPDO, *A2InPDO;
        Delta::rx_t *A1OutPDO, *A2OutPDO;
        int A1ID, A2ID = 0;
        bool A1GapAlarm, A2GapAlarm = false;

        Robot()
        {
            // Set dynamic limits
            input.max_velocity = {1000.0, 1000.0};
            input.max_acceleration = {3000.0, 3000.0};
            input.max_jerk = {10000.0, 10000.0};

            // Set initial conditions
            input.current_position = {0.0, 0.0};
            input.current_velocity = {0.0, 0.0};
            input.current_acceleration = {0.0, 0.0};
            input.target_position = {0.0, 0.0};
            input.target_velocity = {0.0, 0.0};
            input.synchronization = Synchronization::Phase;
        }
        void assignDrives()
        {
            A1OutPDO = (Delta::rx_t *)ec_slave[A1ID].outputs;
            A1InPDO = (Delta::tx_t *)ec_slave[A1ID].inputs;
            A2OutPDO = (Delta::rx_t *)ec_slave[A2ID].outputs;
            A2InPDO = (Delta::tx_t *)ec_slave[A2ID].inputs;

            A1.setCommand(CANOpenCommand::DISABLE);
            A2.setCommand(CANOpenCommand::DISABLE);
        }
        void update();
        bool tracking();
        void commandCb([[maybe_unused]] natsConnection *nc, [[maybe_unused]] natsSubscription *sub, natsMsg *msg,
                       [[maybe_unused]] void *closur);
        std::string to_string() const;
    };
} // namespace FSM
#endif
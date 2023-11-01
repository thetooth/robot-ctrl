#ifndef FSM_ROBOT_HPP
#define FSM_ROBOT_HPP

#include "ethercat.h"
#include "nats.h"
#include "nlohmann/json.hpp"

#include "../CAN/CoEFSM.h"
#include "../common.hpp"
#include "../delta.hpp"

namespace FSM
{
    using json = nlohmann::json;

    enum State
    {
        Idle,
        Halt,
        Startup,
        Homing,
        Tracking,
    };

    class Robot
    {
      public:
        bool run;
        bool estop = true;

        State next = Idle;

        CoEFSM A1;
        CoEFSM A2;

        // PDO memory
        Delta::tx_t *A1InPDO, *A2InPDO;
        Delta::rx_t *A1OutPDO, *A2OutPDO;
        int A1ID, A2ID = 0;
        bool A1GapAlarm, A2GapAlarm = false;

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
        bool home();

        void commandCb([[maybe_unused]] natsConnection *nc, [[maybe_unused]] natsSubscription *sub, natsMsg *msg,
                       [[maybe_unused]] void *closur)
        {
            auto payload = json::parse(natsMsg_GetData(msg));
            if (!payload["command"].is_string())
            {
                spdlog::warn("Invalid JSON received in command handler");
                return;
            }
            auto command = payload["command"].template get<std::string>();

            if (command.compare("start") == 0 && estop)
            {
                run = true;
            }
            if (command.compare("stop") == 0)
            {
                run = false;
            }
            natsMsg_Destroy(msg);
        }

        std::string to_string() const
        {
            switch (next)
            {
            case Idle:
                return "Idle";
            case Halt:
                return "Halt";
            case Startup:
                return "Startup";
            case Homing:
                return "Homing";
            case Tracking:
                return "Tracking";
            default:
                return "[Unknown State]";
            }
        }
    };
} // namespace FSM
#endif
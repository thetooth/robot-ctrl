#ifndef FSM_HPP
#define FSM_HPP

#include "nats.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

enum State
{
    Idle,
    Halt,
    Startup,
    Homing,
    Tracking,
};

class FSM
{
public:
    bool run;
    bool estop;

    State next = Idle;

    CANOpenStateMachine A1;
    CANOpenStateMachine A2;

    // PDO memory
    in_deltab3_t *A1InPDO, *A2InPDO;
    out_deltab3_t *A1OutPDO, *A2OutPDO;

    void init(int a1, int a2)
    {
        assert(a1 != 0 && a2 != 0);
        A1OutPDO = (out_deltab3_t *)ec_slave[a1].outputs;
        A1InPDO = (in_deltab3_t *)ec_slave[a1].inputs;
        A2OutPDO = (out_deltab3_t *)ec_slave[a2].outputs;
        A2InPDO = (in_deltab3_t *)ec_slave[a2].inputs;

        A1.setCommand(CANOpenCommand::DISABLE);
        A2.setCommand(CANOpenCommand::DISABLE);
    }

    void update();

    void commandCb(natsConnection *nc, natsSubscription *sub, natsMsg *msg, void *closur)
    {
        auto payload = json::parse(natsMsg_GetData(msg));
        if (!payload["command"].is_string())
        {
            return;
        }
        auto command = payload["command"].template get<std::string>();

        if (command.compare("start") == 0)
        {
            run = true;
            estop = true;
            wkc += moog_write8(4, 0x6060, 0, 0x8);
            wkc += moog_write8(5, 0x6060, 0, 0x8);
            A1.setCommand(CANOpenCommand::ENABLE);
            A2.setCommand(CANOpenCommand::ENABLE);
        }
        if (command.compare("stop") == 0)
        {
            run = false;
            estop = false;
            A1.setCommand(CANOpenCommand::DISABLE);
            A2.setCommand(CANOpenCommand::DISABLE);
        }
        natsMsg_Destroy(msg);
    }
};

#endif
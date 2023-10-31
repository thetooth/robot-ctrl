#ifndef FSM_HPP
#define FSM_HPP

#include "ethercat.h"
#include "nats.h"
#include "nlohmann/json.hpp"

#include "../common.hpp"
#include "../delta.hpp"
#include "../CAN/CoEFSM.h"

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

    CoEFSM A1;
    CoEFSM A2;

    // PDO memory
    in_deltab3_t *A1InPDO, *A2InPDO;
    out_deltab3_t *A1OutPDO, *A2OutPDO;

    void assignDrives(int a1, int a2)
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

    void commandCb([[maybe_unused]] natsConnection *nc, [[maybe_unused]] natsSubscription *sub, natsMsg *msg, [[maybe_unused]] void *closur)
    {
        auto payload = json::parse(natsMsg_GetData(msg));
        // printf("%s\n", payload.dump().c_str());
        if (!payload["command"].is_string())
        {
            return;
        }
        auto command = payload["command"].template get<std::string>();

        if (command.compare("start") == 0)
        {
            run = true;
            estop = true;
        }
        if (command.compare("stop") == 0)
        {
            run = false;
            estop = false;
        }
        natsMsg_Destroy(msg);
    }
};

#endif
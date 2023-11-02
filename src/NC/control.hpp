#ifndef NC_CONTROL_HPP
#define NC_CONTROL_HPP

#include <thread>

#include "nlohmann/json.hpp"

#include "../FSM/fsm.hpp"

namespace NC
{
    using json = nlohmann::json;

    FSM::Robot *fsmPtr = nullptr;
    void commandCbWrapper(natsConnection *nc, natsSubscription *sub, natsMsg *msg, void *closur)
    {
        if (fsmPtr != nullptr)
        {
            fsmPtr->commandCb(nc, sub, msg, closur);
        }
    }
    void Monitor(FSM::Robot *fsm)
    {
        // Communications
        natsConnection *nc = nullptr;
        natsSubscription *ctrlSub = nullptr;

        fsmPtr = fsm;

        auto ncStatus = natsConnection_ConnectTo(&nc, NATS_DEFAULT_URL);
        if (ncStatus != NATS_OK)
        {
            spdlog::error("NATS connection failure: {}", natsStatus_GetText(ncStatus));
            return;
        }
        ncStatus = natsConnection_Subscribe(&ctrlSub, nc, "motion.command", commandCbWrapper, NULL);
        if (ncStatus != NATS_OK)
        {
            spdlog::error("NATS subscription failure: {}", natsStatus_GetText(ncStatus));
            return;
        }

        // Timing
        struct timespec tick;
        int64_t period = int64_t(16.666666666667e+6 / 2.0);
        int64_t toff = 0;
        int64_t integral = 0;
        clock_gettime(CLOCK_MONOTONIC, &tick);
        TS::Increment(tick, period);

        bool run = true;
        while (run)
        {
            assert(fsm != NULL);

            auto alarm = false;
            if (fsm->A1GapAlarm || fsm->A2GapAlarm)
            {
                alarm = true;
            }

            auto diagStr = std::string("");
            for (auto msg : fsm->diagMsgs)
            {
                diagStr.append(msg + "\n");
            }

            auto [ax, ay] =
                IKScara::forwardKinematics(fsm->A1InPDO->actual_position / GEAR, fsm->A2InPDO->actual_position / GEAR);

            // Status
            json stats = {
                {"run", fsm->run},
                {"alarm", alarm},
                {"state", fsm->to_string()},
                {"diagMsg", diagStr},
                {"dx", ax},
                {"dy", ay},
                {"dAlpha", fsm->A1InPDO->actual_position / GEAR},
                {"dBeta", fsm->A2InPDO->actual_position / GEAR},
            };
            auto payload = stats.dump();
            natsConnection_Publish(nc, "motion.status", payload.c_str(), payload.length());

            if (!fsm->estop && fsm->next == FSM::Idle)
            {
                run = false;
            }

            // calulate toff to get linux time and DC synced
            TS::DCSync(ec_DCtime, CYCLETIME, &integral, &toff);
            // Apply offset to timespec
            TS::AppyOffset(&tick, toff);
            // Monotonic sleep
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
            // Increment timespec by cycletime
            TS::Increment(tick, period);
        }
        natsSubscription_Unsubscribe(ctrlSub);
        natsSubscription_Destroy(ctrlSub);

        natsConnection_FlushTimeout(nc, 1000);
        natsConnection_Close(nc);
        natsConnection_Destroy(nc);

        spdlog::trace("Monitor thread says goodnight");
    };
} // namespace NC

#endif
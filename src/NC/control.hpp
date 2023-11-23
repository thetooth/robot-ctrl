#ifndef NC_CONTROL_HPP
#define NC_CONTROL_HPP

#include <thread>

#include "nlohmann/json.hpp"

#include "../Robot/fsm.hpp"

namespace NC
{
    using json = nlohmann::json;

    Robot::FSM *fsmPtr = nullptr;
    void commandCbWrapper(natsConnection *nc, natsSubscription *sub, natsMsg *msg, void *closur)
    {
        if (fsmPtr != nullptr)
        {
            fsmPtr->receiveCommand(nc, sub, msg, closur);
        }
    }
    void settingsCbWrapper(natsConnection *nc, natsSubscription *sub, natsMsg *msg, void *closur)
    {
        if (fsmPtr != nullptr)
        {
            fsmPtr->receiveSettings(nc, sub, msg, closur);
        }
    }
    void Monitor(Robot::FSM *fsm)
    {
        // Communications
        natsConnection *nc = nullptr;
        natsSubscription *ctrlSub = nullptr;
        natsSubscription *settingsSub = nullptr;

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
        ncStatus = natsConnection_Subscribe(&settingsSub, nc, "motion.settings", settingsCbWrapper, NULL);
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

            fsm->broadcastStatus(nc);

            if (!fsm->estop && fsm->next == Robot::Idle)
            {
                run = false;
            }

            // calculate toff to get linux time and DC synced
            TS::DCSync(ec_DCtime, CYCLETIME, &integral, &toff);
            // Apply offset to timespec
            TS::ApplyOffset(&tick, toff);
            // Monotonic sleep
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
            // Increment timespec by cycle time
            TS::Increment(tick, period);
        }
        natsSubscription_Unsubscribe(ctrlSub);
        natsSubscription_Unsubscribe(settingsSub);
        natsSubscription_Destroy(ctrlSub);
        natsSubscription_Destroy(settingsSub);

        natsConnection_FlushTimeout(nc, 1000);
        natsConnection_Close(nc);
        natsConnection_Destroy(nc);

        spdlog::trace("Monitor thread says goodnight");
    };
} // namespace NC

#endif
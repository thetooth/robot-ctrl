#ifndef NC_CONTROL_HPP
#define NC_CONTROL_HPP

#include <thread>

#include "../FSM/fsm.hpp"

namespace NC
{
    FSM *fsmPtr = nullptr;
    void commandCbWrapper(natsConnection *nc, natsSubscription *sub, natsMsg *msg, void *closur)
    {
        if (fsmPtr != nullptr)
        {
            fsmPtr->commandCb(nc, sub, msg, closur);
        }
    }
    void monitor(FSM *fsm)
    {
        // Communications
        natsConnection *nc = nullptr;
        natsSubscription *ctrlSub = nullptr;

        fsmPtr = fsm;

        auto ncStatus = natsConnection_ConnectTo(&nc, NATS_DEFAULT_URL);
        assert(ncStatus == NATS_OK);
        ncStatus = natsConnection_Subscribe(&ctrlSub, nc, "motion.command", commandCbWrapper, NULL);
        assert(ncStatus == NATS_OK);

        // Timing
        struct timespec tick;
        int64_t period = int64_t(16.666666666667e+6);
        int64_t toff = 0;
        int64_t integral = 0;
        clock_gettime(CLOCK_MONOTONIC, &tick);
        TS::Increment(tick, period);

        for (;;)
        {
            assert(fsm != NULL);
            auto [ax, ay] = IKScara::forwardKinematics(fsm->A1InPDO->actual_position / GEAR, fsm->A2InPDO->actual_position / GEAR);
            // Status
            json stats = {
                {"run", fsm->run},
                {"alarm", false},
                {"dx", ax},
                {"dy", ay},
                {"dAlpha", fsm->A1InPDO->actual_position / GEAR},
                {"dBeta", fsm->A2InPDO->actual_position / GEAR},
            };
            auto payload = stats.dump();
            natsConnection_Publish(nc, "motion.status", payload.c_str(), payload.length());

            // calulate toff to get linux time and DC synced
            TS::DCSync(ec_DCtime, CYCLETIME, &integral, &toff);
            // Apply offset to timespec
            TS::AppyOffset(&tick, toff);
            // Monotonic sleep
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
            // Increment timespec by cycletime
            TS::Increment(tick, period);
        }
    };
}

#endif
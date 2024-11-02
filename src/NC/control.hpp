#ifndef NC_CONTROL_HPP
#define NC_CONTROL_HPP

#include <thread>

#include "nlohmann/json.hpp"

#include "../Robot/fsm.hpp"
#include "kv.hpp"

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
    void settingsKVWatch(kvOperation op, std::string key, std::string value)
    {
        if (op != kvOp_Put)
        {
            return;
        }

        auto payload = json::parse(value);
        spdlog::debug("Settings update: {}", payload.dump());
        std::vector<Robot::OTGSettings> settings;
        for (auto &&axis : payload["data"])
        {
            settings.push_back(axis.template get<Robot::OTGSettings>());
        }

        fsmPtr->updateDynamics(settings);
        fsmPtr->eventLog.Debug(fmt::format("Settings update: {}", key), payload);
    }
    void Monitor(Robot::FSM *fsm)
    {
        Kernel::start_high_latency();

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

        // Settings store
        jsCtx *js = nullptr;
        auto jsStatus = natsConnection_JetStream(&js, nc, NULL);
        if (jsStatus != NATS_OK)
        {
            spdlog::error("Failed to get JetStream context");
            return;
        }

        auto settingsKV = KV(js, "setting");

        std::thread settingsKVThread(&KV::watch, settingsKV, "dynamics.active", settingsKVWatch);

        // Timing
        struct timespec tick;
        int64_t period = int64_t((1.0 / 250.0) * TS::NSEC_PER_SECOND);
        int64_t toff = 0;
        int64_t integral = 0;
        clock_gettime(CLOCK_MONOTONIC, &tick);
        TS::Increment(tick, period);

        bool run = true;
        while (run)
        {
            assert(fsm != NULL);

            fsm->broadcastStatus(nc);

            if (fsm->shutdown)
            {
                run = false;
            }

            // calculate toff to get linux time and DC synced
            TS::DCSync(ec_DCtime, SYNC0, &integral, &toff);
            // Apply offset to timespec
            TS::ApplyOffset(&tick, toff);
            // Monotonic sleep
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
            // Increment timespec by cycle time
            TS::Increment(tick, period);
        }

        settingsKVThread.join();

        natsSubscription_Unsubscribe(ctrlSub);
        natsSubscription_Destroy(ctrlSub);

        natsConnection_FlushTimeout(nc, 1000);
        natsConnection_Close(nc);
        natsConnection_Destroy(nc);

        spdlog::trace("Monitor thread says goodnight");
    };
} // namespace NC

#endif
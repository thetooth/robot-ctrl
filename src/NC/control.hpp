#ifndef NC_CONTROL_HPP
#define NC_CONTROL_HPP

#include <thread>

#include "nlohmann/json.hpp"

#include "../Robot/fsm.hpp"
#include "kv.hpp"

namespace NC
{
    using json = nlohmann::json;

    void Monitor(std::string url, Robot::FSM *fsm)
    {
        Kernel::start_high_latency();

        // Communications
        natsConnection *nc = nullptr;
        natsSubscription *ctrlSub = nullptr;

        auto ncStatus = natsConnection_ConnectTo(&nc, url.c_str());
        if (ncStatus != NATS_OK)
        {
            spdlog::error("NATS connection failure: {}", natsStatus_GetText(ncStatus));
            return;
        }
        ncStatus = natsConnection_Subscribe(
            &ctrlSub, nc, "motion.command",
            []([[maybe_unused]] natsConnection *nc, [[maybe_unused]] natsSubscription *sub, natsMsg *msg,
               void *closure) {
                auto fsm = static_cast<Robot::FSM *>(closure);
                auto payload = json::parse(natsMsg_GetData(msg));

                try
                {
                    fsm->receiveCommand(payload);
                }
                catch (const json::parse_error &e)
                {
                    spdlog::error("commandCb parsing error: {}", e.what());
                }
                catch (const json::exception &e)
                {
                    spdlog::error("commandCb exception: {}", e.what());
                }

                natsMsg_Destroy(msg);
            },
            fsm);
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

        std::thread settingsKVThread(&KV::watch, settingsKV, "dynamics.active",
                                     [fsm](kvOperation op, std::string key, std::string value) {
                                         if (op != kvOp_Put)
                                         {
                                             return;
                                         }

                                         try
                                         {
                                             auto payload = json::parse(value);
                                             auto settings = payload.get<Robot::Preset>();

                                             fsm->updateDynamics(settings);
                                             fsm->eventLog.Debug(fmt::format("Settings update: {}", key), payload);
                                         }
                                         catch (const json::parse_error &e)
                                         {
                                             spdlog::error("Settings parsing error: {}", e.what());
                                         }
                                         catch (const json::exception &e)
                                         {
                                             spdlog::error("Settings exception: {}", e.what());
                                         }
                                     });

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
#pragma once

#include <thread>

#include "nats.h"
#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"

#include "../Robot/fsm.hpp"
#include "kv.hpp"

namespace NC
{
    using json = nlohmann::json;

    class Manager
    {
      public:
        Manager(std::string url, std::unique_ptr<Robot::FSM> fsm) : fsm(std::move(fsm))
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
                fsm.get());
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
            // Main loop
            while (true)
            {
                if (fsm->shutdown)
                {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // Cleanup
            kv.close();
            natsSubscription_Destroy(ctrlSub);
            natsConnection_Destroy(nc);
        }
        ~Manager() = default;

        void settingsKVWatch(kvOperation op, std::string key, std::string value)
        {
            if (op != kvOp_Put)
            {
                return;
            }

            auto payload = json::parse(value);
            spdlog::debug("Settings update: {}", payload.dump());

            auto settings = payload.get<Robot::Preset>();

            fsm->updateDynamics(settings);
            fsm->eventLog.Debug(fmt::format("Settings update: {}", key), payload);
        }

      private:
        std::unique_ptr<Robot::FSM> fsm;
    };
} // namespace NC
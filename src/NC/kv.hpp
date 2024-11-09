#ifndef NC_KV_HPP
#define NC_KV_HPP

#include <nats.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace NC
{
    //! @brief Key-Value store callback
    typedef void(Callback)(kvOperation op, std::string key, std::string value);

    //! @brief Key-Value store
    //!
    //! This class provides a simple interface to the NATS key-value store.
    class KV
    {
      public:
        KV(jsCtx *js, std::string bucket)
        {
            auto jsStatus = js_KeyValue(&store, js, bucket.c_str());
            if (jsStatus != NATS_OK)
            {
                spdlog::error("Failed to get key-value store: {}", natsStatus_GetText(jsStatus));
            }
        }
        ~KV()
        {
            if (watcher != nullptr)
            {
                kvWatcher_Stop(watcher);
                kvWatcher_Destroy(watcher);
            }
            if (store != nullptr)
            {
                kvStore_Destroy(store);
            }
        }

        natsStatus put(std::string key, std::string value)
        {
            return kvStore_Put(NULL, store, key.c_str(), value.c_str(), value.size());
        }

        natsStatus get(std::string key, std::string &value)
        {
            kvEntry *entry = nullptr;
            natsStatus ncStatus = kvStore_Get(&entry, store, key.c_str());
            if (ncStatus == NATS_OK)
            {
                value = kvEntry_ValueString(entry);
            }
            kvEntry_Destroy(entry);

            return ncStatus;
        }

        natsStatus del(std::string key)
        {
            return kvStore_Delete(store, key.c_str());
        }

        natsStatus watch(std::string wildcard, std::function<Callback> cb)
        {
            auto ncStatus = kvStore_Watch(&watcher, store, wildcard.c_str(), NULL);

            if (ncStatus != NATS_OK)
            {
                spdlog::error("Failed to watch key-value store: {}", natsStatus_GetText(ncStatus));
                return ncStatus;
            }

            return receive(cb);
        }

        natsStatus watchAll(std::function<Callback> cb)
        {
            auto ncStatus = kvStore_WatchAll(&watcher, store, NULL);
            if (ncStatus != NATS_OK)
            {
                spdlog::error("Failed to watch key-value store: {}", natsStatus_GetText(ncStatus));
                return ncStatus;
            }

            return receive(cb);
        }

        natsStatus receive(std::function<Callback> cb)
        {
            closed = false;
            natsStatus ncStatus = NATS_OK;

            while (!closed)
            {
                kvEntry *entry = nullptr;
                ncStatus = kvWatcher_Next(&entry, watcher, 10);
                if (ncStatus == NATS_OK)
                {
                    std::string key = "";
                    std::string value = "";
                    if (kvEntry_Key(entry) != NULL)
                    {
                        key = kvEntry_Key(entry);
                    }
                    if (kvEntry_Operation(entry) == kvOp_Put)
                    {
                        value = kvEntry_ValueString(entry);
                    }
                    cb(kvEntry_Operation(entry), key, value);
                }
                else if (ncStatus == NATS_TIMEOUT)
                {
                    kvEntry_Destroy(entry);
                    continue;
                }
                else
                {
                    spdlog::error("Failed to watch key-value store: {}", natsStatus_GetText(ncStatus));
                }
                kvEntry_Destroy(entry);
            }

            kvWatcher_Destroy(watcher);

            return ncStatus;
        }

        void close()
        {
            closed = true;
            if (watcher != nullptr)
            {
                kvWatcher_Stop(watcher);
            }
        }

      private:
        bool closed = false;
        kvStore *store = nullptr;
        kvWatcher *watcher = nullptr;
    };
} // namespace NC
#endif // NC_KV_HPP
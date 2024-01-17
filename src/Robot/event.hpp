#ifndef ROBOT_EVENT_HPP
#define ROBOT_EVENT_HPP

#include <ctime>
#include <deque>
#include <random>
#include <string>

#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"

namespace Robot
{
    using json = nlohmann::json;

    int64_t get_timestamp();
    std::string get_iso8601();
    int get_random_number(int lower, int upper);
    std::string generate_uuid();

    class Event
    {
      public:
        std::string id;
        enum Level
        {
            Debug,
            Info,
            Warning,
            Error,
            Critical,
            Kinematic,
            EtherCAT,
        } level;
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> time;
        std::string message;

        Event(Level level, std::string message)
            : id(generate_uuid()), level(level), time(std::chrono::system_clock::now()), message(message)
        {
        }

        std::string levelToString() const;
    };

    void to_json(json &j, const Event &e);

    class EventLog : public std::deque<Event>
    {
      public:
        EventLog() : std::deque<Event>()
        {
        }

        void Debug(std::string message)
        {
            spdlog::debug(message);
            push_back(Event(Event::Level::Debug, message));
        }
        void Info(std::string message)
        {
            spdlog::info(message);
            push_back(Event(Event::Level::Info, message));
        }
        void Warning(std::string message)
        {
            spdlog::warn(message);
            push_back(Event(Event::Level::Warning, message));
        }
        void Error(std::string message)
        {
            spdlog::error(message);
            push_back(Event(Event::Level::Error, message));
        }
        void Critical(std::string message)
        {
            spdlog::critical(message);
            push_back(Event(Event::Level::Critical, message));
        }
        void Kinematic(std::string message)
        {
            push_back(Event(Event::Level::Kinematic, message));
        }
        void EtherCAT(std::string message)
        {
            push_back(Event(Event::Level::EtherCAT, message));
        }
    };

} // namespace Robot

#endif // ROBOT_EVENT_HPP
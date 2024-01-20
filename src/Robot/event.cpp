#include "event.hpp"

std::string Robot::Event::levelToString() const
{
    switch (level)
    {
    case Level::Debug:
        return "debug";
    case Level::Info:
        return "info";
    case Level::Warning:
        return "warning";
    case Level::Error:
        return "error";
    case Level::Critical:
        return "critical";
    case Level::Kinematic:
        return "kinematic";
    case Level::EtherCAT:
        return "ethercat";
    default:
        return "unknown";
    }
}

void Robot::to_json(json &j, const Event &e)
{
    j = json{{"id", e.id}, {"level", e.levelToString()}, {"time", get_timestamp()}, {"msg", e.message}};
    if (!e.detail.is_null())
    {
        j["detail"] = e.detail;
    }
}

int64_t Robot::get_timestamp()
{
    const auto tp = std::chrono::system_clock::now();
    const auto dur = tp.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(dur).count();
}

std::string Robot::get_iso8601()
{
    std::time_t now;
    std::time(&now);
    char buf[sizeof "2011-10-08T07:07:09Z"];
    std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&now));
    return buf;
}

int Robot::get_random_number(int lower, int upper)
{
    static std::random_device random_device;
    static std::default_random_engine random_engine(random_device());
    static std::uniform_int_distribution<int> uniform_dist(lower, upper);
    return uniform_dist(random_engine);
}

std::string Robot::generate_uuid()
{
    std::string result(32, ' ');

    for (std::size_t i = 0; i < result.size(); ++i)
    {
        // the 12th character must be a '4'
        if (i == 12)
        {
            result[i] = '4';
        }
        else
        {
            // get a random number from 0..15
            const auto r = static_cast<char>(get_random_number(0, 15));
            if (r < 10)
            {
                result[i] = '0' + r;
            }
            else
            {
                result[i] = 'a' + r - static_cast<char>(10);
            }
        }
    }

    return result;
}
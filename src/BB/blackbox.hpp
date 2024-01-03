#ifndef BB_BLACKBOX_HPP
#define BB_BLACKBOX_HPP

#include <thread>

#include "nlohmann/json.hpp"

#include "../Robot/fsm.hpp"
#include "../gnuplot.hpp"

namespace BB
{
    using json = nlohmann::json;

    Robot::FSM *fsmPtr = nullptr;

    void Monitor(Robot::FSM *fsm)
    {
        // Plotting
        GnuplotPipe gp;
        auto buffered = true;
        gp.sendLine("set terminal png size 4096,1920", buffered);
        gp.sendLine("set key", buffered);
        gp.sendLine("set output 'test.png'", buffered);
        gp.sendLine("$DATA << EOD", buffered);
        gp.sendLine("sync0 comp integral", buffered);

        // Timing
        struct timespec tick;
        int64_t period = CYCLETIME;
        int64_t toff = 0;
        int64_t integral = 0;
        clock_gettime(CLOCK_MONOTONIC, &tick);
        TS::Increment(tick, period);

        bool run = true;
        while (run)
        {
            assert(fsm != NULL);

            if (!fsm->estop)
            {
                run = false;
            }

            gp.sendLine(fmt::format("{} {} {}", fsm->status.ethercat.sync0, fsm->status.ethercat.compensation,
                                    fsm->status.ethercat.integral),
                        buffered);

            // calculate toff to get linux time and DC synced
            TS::DCSync(ec_DCtime, CYCLETIME, &integral, &toff);
            // Apply offset to timespec
            TS::ApplyOffset(&tick, toff);
            // Monotonic sleep
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
            // Increment timespec by cycle time
            TS::Increment(tick, period);
        }

        // gp.sendEndOfData();
        gp.sendLine("EOD", buffered);
        gp.sendLine("plot for [col=1:3] $DATA using 0:col with lines title columnheader", buffered);
        gp.writeBufferToFile("plot.txt");

        spdlog::trace("Blackbox says goodnight");
    }
} // namespace BB

#endif
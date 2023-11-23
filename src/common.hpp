#ifndef _COMMON_H
#define _COMMON_H

#include <cstdint>

#include "ethercat.h"

#define GEAR (10.0 * 3.44) // Gearbox * pulley
#define PPU (46603.0)      // Units per degree
#define PPV (10.0 / 6.0)   // Units per degree per second
#define SYNC0 1e6
#define CYCLETIME SYNC0

namespace TS
{
    static const int NSEC_PER_SECOND = 1e+9;
    static const int USEC_PER_SECOND = 1e+6;

    [[maybe_unused]] static void DCSync(int64_t reftime, int64_t cycletime, int64_t *integral, int64_t *offsettime)
    {
        // static int64_t integral;
        int64_t delta;
        /* set linux sync point 500us later than DC sync, just as example */
        delta = (reftime - 500000) % cycletime;
        if (delta > (cycletime / 2))
        {
            delta = delta - cycletime;
        }
        if (delta > 0)
        {
            (*integral)++;
        }
        if (delta < 0)
        {
            (*integral)--;
        }
        *offsettime = -(delta / 100) - (*integral / 20);
    }

    [[maybe_unused]] static void ApplyOffset(struct timespec *ts, int64 addtime)
    {
        int64 sec, nsec;

        nsec = addtime % NSEC_PER_SECOND;
        sec = (addtime - nsec) / NSEC_PER_SECOND;
        ts->tv_sec += sec;
        ts->tv_nsec += nsec;
        if (ts->tv_nsec > NSEC_PER_SECOND)
        {
            nsec = ts->tv_nsec % NSEC_PER_SECOND;
            ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SECOND;
            ts->tv_nsec = nsec;
        }
    }

    [[maybe_unused]] static void Increment(struct timespec &tick, int64_t nsec)
    {
        tick.tv_nsec += nsec;
        while (tick.tv_nsec >= NSEC_PER_SECOND)
        {
            tick.tv_nsec -= NSEC_PER_SECOND;
            tick.tv_sec++;
        }
    }
} // namespace TS

#endif
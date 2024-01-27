#ifndef _COMMON_H
#define _COMMON_H

#include <cstdint>

#include "ethercat.h"
#include "spdlog/spdlog.h"

#define GEAR (50.0)      // 50:1 Harmonic drive
#define PPU (46603.0)    // Units per degree
#define PPV (10.0 / 6.0) // Units per degree per second
#define SYNC0 1e6
#define CYCLETIME SYNC0

namespace TS
{
    static const int NSEC_PER_SECOND = 1e+9;
    static const int USEC_PER_SECOND = 1e+6;

    //! @brief Calculate DC sync offset
    //!
    //! This function uses a simple PI controller to try and align the Linux clock with the DC sync0 event.
    [[maybe_unused]] static void DCSync(int64_t reftime, int64_t cycletime, int64_t *integral, int64_t *offsettime)
    {
        /* set linux sync point 500us later than DC sync, just as example */
        int64_t delta = (reftime - 50000) % cycletime;
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
        *offsettime = -(delta / 100) - (*integral / 20); // Original 100 / 20
    }

    //! @brief Calculate DC sync offset
    //!
    //! This function applies the offset calculated by DCSync to the timespec struct.
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

    //! @brief Increment timespec by nsec
    //!
    //! This function increments the timespec struct by nsec. If the value exceeds 1 second, the seconds value is
    //! incremented.
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

namespace Kernel
{
    static int pm_qos_fd = -1;

    [[maybe_unused]] static void start_low_latency(void)
    {
        // Set realtime priority
        struct sched_param schedularParam;
        memset(&schedularParam, 0, sizeof(schedularParam));
        // Do not set priority above 49, otherwise sockets are starved
        schedularParam.sched_priority = 30;
        if (sched_setscheduler(0, SCHED_FIFO, &schedularParam) == -1)
        {
            spdlog::critical("Failed to set realtime priority: {}", strerror(errno));
            exit(errno);
        }

        // Set CPU affinity
        cpu_set_t cpuSet;
        CPU_ZERO(&cpuSet);
        // Use both isolated cores of Intel Atom x7425E as they share a single L2 cache,
        // the cache miss leads to large clock skew when the other core performs a load
        // from main memory, or enters a low power state.
        CPU_SET(2, &cpuSet);
        // CPU_SET(3, &cpuSet);
        if (sched_setaffinity(getpid(), sizeof(cpuSet), &cpuSet) == -1)
        {
            spdlog::critical("Failed to set CPU affinity: {}", strerror(errno));
            exit(errno);
        }

        // Check if cpu_dma_latency file is already open
        int32_t target = 0;
        if (pm_qos_fd >= 0)
        {
            return;
        }

        // Open cpu_dma_latency file, set to 0us.
        // This prevents the CPU from entering a low power state.
        pm_qos_fd = open("/dev/cpu_dma_latency", O_RDWR);
        if (pm_qos_fd < 0)
        {
            spdlog::critical("Failed to open PM QOS file: {}", strerror(errno));
            exit(errno);
        }
        write(pm_qos_fd, &target, sizeof(target));
    }

    [[maybe_unused]] static void stop_low_latency(void)
    {
        if (pm_qos_fd >= 0)
        {
            close(pm_qos_fd);
        }
    }

    [[maybe_unused]] static void start_high_latency(void)
    {
        // Set low priority
        struct sched_param schedularParam;
        memset(&schedularParam, 0, sizeof(schedularParam));
        schedularParam.sched_priority = 10;
        if (sched_setscheduler(getpid(), SCHED_FIFO, &schedularParam) == -1)
        {
            spdlog::critical("Failed to set priority: {}", strerror(errno));
            exit(errno);
        }

        // Set CPU affinity
        cpu_set_t cpuSet;
        CPU_ZERO(&cpuSet);
        // Use both isolated cores of Intel Atom x7425E as they share a single L2 cache,
        // the cache miss leads to large clock skew when the other core performs a load
        // from main memory, or enters a low power state.
        // CPU_SET(0, &cpuSet);
        CPU_SET(3, &cpuSet);
        if (sched_setaffinity(getpid(), sizeof(cpuSet), &cpuSet) == -1)
        {
            spdlog::critical("Failed to set CPU affinity: {}", strerror(errno));
            exit(errno);
        }
    }
} // namespace Kernel

#endif
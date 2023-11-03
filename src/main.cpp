#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <sys/resource.h>
#include <sys/time.h>

#include <chrono>
#include <math.h>
#include <sanitizer/lsan_interface.h>
#include <signal.h>
#include <thread>

#include "ethercat.h"
#include "osal.h"
#include "oshw.h"
#include "ruckig/ruckig.hpp"
#include "spdlog/spdlog.h"

#include "FSM/fsm.hpp"
#include "NC/control.hpp"

// EtherCAT variables:
char IOmap[4096];
unsigned int usedmem;

// FSM
auto fsm = FSM::Robot();

auto priorAbort = false;
void abort_handler([[maybe_unused]] int signum)
{
    printf("\n");
    spdlog::critical("Aborting due to SIGINT");
    fsm.estop = false;
    if (priorAbort)
    {
        exit(255);
    }
    priorAbort = true;
}

int nic_setup(char *ifname)
{
    // Initialize SOEM, bind socket to ifname
    if (!ec_init(ifname))
    {
        spdlog::critical("Could not initialize NIC {}, maybe it's unplugged...", ifname);
        return 1;
    }

    // Find and auto-config slaves
    if (!ec_config_init(FALSE))
    {
        spdlog::critical("Could not auto configure slaves");
        return 1;
    }

    spdlog::info("{} slaves found and configured", ec_slavecount);

    // Map CoE drives
    for (auto cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        auto &&slave = ec_slave[cnt];
        if (std::strcmp(slave.name, "ASDA-B3-E CoE Drive") == 0)
        {
            if (fsm.A1ID == 0)
            {
                spdlog::debug("Assign {} {} as A1", slave.name, cnt);
                fsm.A1ID = cnt;
            }
            else if (fsm.A2ID == 0)
            {
                spdlog::debug("Assign {} {} as A2", slave.name, cnt);
                fsm.A2ID = cnt;
            }
        }
    }

    if (fsm.A1ID == 0 || fsm.A2ID == 0)
    {
        spdlog::critical("One or more drives are missing");
        return 1;
    }

    // Drive startup params
    ec_slave[fsm.A1ID].PO2SOconfig = Delta::PO2SOconfig;
    ec_slave[fsm.A2ID].PO2SOconfig = Delta::PO2SOconfig;

    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

    ec_configdc();
    ec_readstate();

    // DC Setup
    ec_dcsync0(fsm.A1ID, true, SYNC0, 0);
    ec_dcsync0(fsm.A2ID, true, SYNC0, 0);

    usedmem = ec_config_map(&IOmap);
    if (!(usedmem <= sizeof(IOmap)))
    {
        spdlog::critical("IO Map not big enough");
        return 1;
    }

    spdlog::info("Slaves mapped and configured");

    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    spdlog::debug("DC capable: {}", (ec_configdc() ? "yes" : "no :("));

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    // send one valid process data to make outputs in slaves happy
    ec_send_processdata();
    Common::wkc = ec_receive_processdata(EC_TIMEOUTRET);

    ec_writestate(0);
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    } while (ec_slave[0].state != EC_STATE_OPERATIONAL);
    // wait for all slaves to reach OP state

    return 0;
}

int main()
{
    spdlog::set_level(spdlog::level::debug);
    spdlog::set_pattern("%^[%=8l]%$ %s:%# %v");

    signal(SIGINT, abort_handler);

    // Set realtime priority
    struct sched_param schedp;
    memset(&schedp, 0, sizeof(schedp));
    // Do not set priority above 49, otherwise sockets are starved
    schedp.sched_priority = 30;
    sched_setscheduler(0, SCHED_FIFO, &schedp);

    // Setup EtherCAT interface:
    if (nic_setup(const_cast<char *>("enp2s0")) == 1)
    {
        spdlog::critical("Setup encountered an error and cannot continue");
        return 1;
    }

    // Assign slave ids and setup PDO table
    fsm.assignDrives();

    // Setup message bus
    auto monitor = std::thread(NC::Monitor, &fsm);

    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        spdlog::info("Operational state reached for all slaves.");

        // Timing
        struct timespec tick;
        int64_t toff = 0;
        int64_t integral = 0;
        clock_gettime(CLOCK_MONOTONIC, &tick);
        TS::Increment(tick, CYCLETIME);

        // Cyclic loop
        while (true)
        {
            ec_send_processdata();
            Common::wkc = ec_receive_processdata(EC_TIMEOUTRET);

            fsm.update();
            if (!fsm.estop && fsm.next == FSM::Idle)
            {
                monitor.join();
                ec_close();
#ifdef WITH_ADDR_SANITIZE
                __lsan_do_leak_check();
#endif
                return 1;
            }

            // spdlog::debug("Offset: {}nS", toff);

            // calculate toff to get linux time and DC synced
            TS::DCSync(ec_DCtime, CYCLETIME, &integral, &toff);
            // Apply offset to timespec
            TS::ApplyOffset(&tick, toff);
            // Monotonic sleep
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
            // Increment timespec by cycle time
            TS::Increment(tick, CYCLETIME);
        }
    }

    return 0;
}
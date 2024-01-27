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

#include "BB/blackbox.hpp"
#include "NC/control.hpp"
#include "Robot/Drive/delta.hpp"
#include "Robot/Drive/sim.hpp"
#include "Robot/fsm.hpp"

#include "gnuplot.hpp"

using namespace std::literals::chrono_literals;

// EtherCAT variables:
char IOmap[4096];
unsigned int usedmem;
int J1ID, J2ID, J3ID, J4ID;

// FSM
auto fsm = Robot::FSM();

auto priorAbort = false;
constexpr std::chrono::nanoseconds HALT_TIMEOUT = 1s;
std::chrono::nanoseconds haltTimestamp = 0ns;

//! @brief Abort handler
//!
//! This function is called when the program receives a SIGINT signal. It sets the estop flag to false
//! and forcibly terminates the program if a prior attempt was already made.
void abort_handler([[maybe_unused]] int signum)
{
    printf("\n");
    fsm.eventLog.Critical("Aborting due to SIGINT");
    fsm.estop = false;
    if (priorAbort)
    {
        exit(255);
    }
    haltTimestamp = std::chrono::system_clock::now().time_since_epoch();
    priorAbort = true;
}

//! @brief Setup EtherCAT interface
//!
//! This function initializes the EtherCAT network interface and configures the slaves.
//! It also sets up the DC sync0 event and starts the DC timer.
//!
//! @param ifname The name of the network interface to bind to
//! @return 0 on success, 1 on failure
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

    fsm.eventLog.EtherCAT(fmt::format("{} slaves found and configured", ec_slavecount));

    // Map CoE drives
    for (auto cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        auto &&slave = ec_slave[cnt];
        if (std::strcmp(slave.name, "ASDA-B3-E CoE Drive") == 0)
        {
            if (J1ID == 0)
            {
                fsm.eventLog.EtherCAT(fmt::format("Assign {} {} as J1", slave.name, cnt));
                J1ID = cnt;
            }
            else if (J2ID == 0)
            {
                fsm.eventLog.EtherCAT(fmt::format("Assign {} {} as J2", slave.name, cnt));
                J2ID = cnt;
            }
        }
    }

    if (J1ID == 0 || J2ID == 0)
    {
        spdlog::critical("One or more drives are missing");
        return 1;
    }

    // Drive startup params
    ec_slave[J1ID].PO2SOconfig = Delta::PO2SOconfig;
    ec_slave[J2ID].PO2SOconfig = Delta::PO2SOconfig;

    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

    ec_configdc();
    ec_readstate();

    // DC Setup
    ec_dcsync0(J1ID, true, SYNC0, 0);
    ec_dcsync0(J2ID, true, SYNC0, 0);

    usedmem = ec_config_map(&IOmap);
    if (!(usedmem <= sizeof(IOmap)))
    {
        spdlog::critical("IO Map not big enough");
        return 1;
    }

    fsm.eventLog.EtherCAT("Slaves mapped and DC synchronization started");

    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    fsm.eventLog.Debug(fmt::format("DC capable: {}", (ec_configdc() ? "yes" : "no :(")));

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    // send one valid process data to make outputs in slaves happy
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

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
    signal(SIGTERM, abort_handler);
    signal(SIGKILL, abort_handler);
    signal(SIGSTOP, abort_handler);

    // Set thread policy and CPU affinity, this also locks the processor in high power state
    Kernel::start_low_latency();

    // Setup EtherCAT interface:
    if (nic_setup(const_cast<char *>("enp2s0")) == 1)
    {
        spdlog::critical("Setup encountered an error and cannot continue");
        return 1;
    }

    // Check if all slaves to reach OP state
    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        fsm.eventLog.EtherCAT("Operational state reached for all slaves");
    }
    else
    {
        spdlog::critical("Not all slaves reached operational state.");
        return 1;
    }

    // Setup drive PDO objects
    Delta::PDO J1(J1ID);
    Delta::PDO J2(J2ID);
    Sim::PDO J3;
    Sim::PDO J4;

    // Assign slave ids and setup PDO table
    fsm.J1 = Drive::Motor{J1ID, &J1, PPU * GEAR, PPV * GEAR, -65, 245};
    fsm.J2 = Drive::Motor{J2ID, &J2, PPU * GEAR, PPV * GEAR, -155, 155};
    fsm.J3 = Drive::Motor{3, &J3, PPU, PPV, -3600, 3600};
    fsm.J4 = Drive::Motor{4, &J4, PPU, PPV, -360, 360};
    // Assign drive groups
    fsm.Arm = Drive::Group{&fsm.J1, &fsm.J2, &fsm.J3, &fsm.J4};

    // Setup message bus
    auto monitor = std::thread(NC::Monitor, &fsm);

    // Working counter
    auto expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    auto wkc = 0;
    fsm.eventLog.Debug(fmt::format("Expected WKC {}", expectedWKC));

    fsm.Arm.setTorqueLimit(20);
    fsm.Arm.setTorqueThreshold(15);
    fsm.Arm.setFollowingWindow(300);

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
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (fsm.estop && wkc < expectedWKC)
        {
            if (wkc == -1)
            {
                fsm.eventLog.Critical("EtherCAT bus failure");
            }
            else
            {
                fsm.eventLog.Critical(fmt::format("WKC less than expected {} < {}", wkc, expectedWKC));
            }
            fsm.estop = false;
        }

        fsm.update();
        if (!fsm.estop && (fsm.next == Robot::Idle ||
                           (std::chrono::system_clock::now().time_since_epoch() - haltTimestamp) > HALT_TIMEOUT))
        {
            spdlog::critical("Halting");
            monitor.join();

            ec_close();
            Kernel::stop_low_latency();
#ifdef WITH_ADDR_SANITIZE
            __lsan_do_leak_check();
#endif
            return 1;
        }

        fsm.status.ethercat = {
            .interval = int64_t(CYCLETIME),
            .sync0 = ec_DCtime % int64_t(CYCLETIME),
            .compensation = toff,
            .integral = integral,
        };

        // calculate toff to get linux time and DC synced
        TS::DCSync(ec_DCtime, CYCLETIME, &integral, &toff);
        // Apply offset to timespec
        TS::ApplyOffset(&tick, toff);
        // Monotonic sleep
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
        // Increment timespec by cycle time
        TS::Increment(tick, CYCLETIME);
    }

    return 0;
}
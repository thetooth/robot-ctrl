#include "ethercat.h"
#include "osal.h"
#include "oshw.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <sys/time.h>
#include <sys/resource.h>

#include <math.h>
#include <chrono>
#include <thread>

#include "ruckig/ruckig.hpp"

#include "common.hpp"
#include "IK/scara.hpp"
#include "FSM/fsm.hpp"
#include "NC/control.hpp"

using namespace std::literals::chrono_literals;
using namespace std::chrono;
using namespace ruckig;
using json = nlohmann::json;

// EtherCAT variables:
char IOmap[4096];
unsigned int usedmem;

int A1ID = 0;
int A2ID = 0;
bool A1GapAlarm, A2GapAlarm = false;

// FSM
auto fsm = FSM();

int nic_setup(char *ifname)
{
    // initialise SOEM, bind socket to ifname
    if (!ec_init(ifname))
    {
        return 1;
    }

    // find and auto-config slaves
    if (!ec_config_init(FALSE))
    {
        return 1;
    }

    printf("%d slaves found and configured.\n", ec_slavecount);

    // Map CoE drives
    for (auto cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        auto &&slave = ec_slave[cnt];
        if (std::strcmp(slave.name, "ASDA-B3-E CoE Drive") == 0)
        {
            if (A1ID == 0)
            {
                printf("Assign %s %d as A1\n", slave.name, cnt);
                A1ID = cnt;
            }
            else if (A2ID == 0)
            {
                printf("Assign %s %d as A2\n", slave.name, cnt);
                A2ID = cnt;
            }
        }
    }

    assert(A1ID != 0);
    assert(A2ID != 0);

    // Drive startup params
    ec_slave[A1ID].PO2SOconfig = Delta::PO2SOconfig;
    ec_slave[A2ID].PO2SOconfig = Delta::PO2SOconfig;

    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

    ec_configdc();
    ec_readstate();

    // DC Setup
    ec_dcsync0(A1ID, true, SYNC0, 3000);
    ec_dcsync0(A2ID, true, SYNC0, 3000);

    usedmem = ec_config_map(&IOmap);
    if (!(usedmem <= sizeof(IOmap)))
    {
        printf("IO Map not big enough");
    }

    printf("Slaves mapped and configured.\n");

    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    printf("DC capable : %s\n", (ec_configdc() ? "yes" : "no :("));

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
    struct sched_param schedp;
    memset(&schedp, 0, sizeof(schedp));
    /* do not set priority above 49, otherwise sockets are starved */
    schedp.sched_priority = 30;
    sched_setscheduler(0, SCHED_FIFO, &schedp);

    // Setup EtherCAT interface:
    if (nic_setup(const_cast<char *>("enp2s0")) == 1)
    {
        // Setup encountered and error and cannot continue.
        return 1;
    }

    // Assign slave ids and setup PDO table
    fsm.assignDrives(A1ID, A2ID);

    // Setup message bus
    auto monitor = std::thread(NC::monitor, &fsm);

    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        printf("Operational state reached for all slaves.\n");
        auto inSync = FALSE;

        auto dx = 110.0;
        auto dy = 170.0;
        auto pathPos = 0;

        // Create instances: the Ruckig OTG as well as input and output parameters
        Ruckig<2> otg{0.002}; // control cycle
        InputParameter<2> input;
        OutputParameter<2> output;

        input.max_velocity = {1000.0, 1000.0};
        input.max_acceleration = {1000.0, 1000.0};
        input.max_jerk = {1000.0, 1000.0};
        // Set input parameters
        input.current_position = {0.0};
        input.current_velocity = {0.0};
        input.current_acceleration = {0.0};

        input.target_position = {0.0, 0.0};
        input.target_velocity = {0.0, 0.0};

        // Timing
        struct timespec tick;
        int64_t toff = 0;
        int64_t integral = 0;
        clock_gettime(CLOCK_MONOTONIC, &tick);
        TS::Increment(tick, CYCLETIME);

        /* cyclic loop */
        while (true)
        {
            ec_send_processdata();
            Common::wkc = ec_receive_processdata(EC_TIMEOUTRET);

            // printf("status: 0x%x 0x%x\n",
            //        A1InPDO->status_word,
            //        A2InPDO->status_word);

            fsm.update();
            if (fsm.next == Tracking)
            {
                if (!inSync)
                {
                    // Set input parameters
                    input.current_position = {
                        fsm.A1InPDO->actual_position / GEAR,
                        fsm.A2InPDO->actual_position / GEAR,
                    };
                    input.current_velocity = {0.0, 0.0};
                    input.current_acceleration = {0.0, 0.0};
                    otg.reset();
                    // otg.update(input, output);
                    // fsm.A1OutPDO->target_position = fsm.A1InPDO->actual_position;
                    // fsm.A2OutPDO->target_position = fsm.A2InPDO->actual_position;

                    printf("Resync\n");
                    inSync = TRUE;
                }
                auto [theta1, theta2, phi] = IKScara::inverseKinematics(dx, dy);

                input.target_position[0] = theta1;
                input.target_position[1] = theta2;

                auto res = otg.update(input, output);
                if (res == ruckig::Finished)
                {
                    switch (pathPos)
                    {
                    case 0:
                    default:
                        pathPos = 0;
                        dx = 0;
                        dy = 300;
                        break;
                    case 1:
                        dx = -120;
                        dy = 36;
                        break;
                    case 2:
                        dx = 136;
                        dy = -50;
                        break;
                    }
                    pathPos++;
                }
                auto &p = output.new_position;

                fsm.A1OutPDO->target_position = IKScara::gap(p[0], fsm.A1InPDO->actual_position / GEAR, &A1GapAlarm) * GEAR;
                fsm.A2OutPDO->target_position = IKScara::gap(p[1], fsm.A2InPDO->actual_position / GEAR, &A2GapAlarm) * GEAR;

                // Set input parameters
                output.pass_to_input(input);

                // printf("positon: %f %f %f %f\n", theta1, theta2, p[0], p[1]);
            }
            else
            {
                inSync = false;
            }

            // std::cout << "Offset: " << toff << std::endl;

            // calulate toff to get linux time and DC synced
            TS::DCSync(ec_DCtime, CYCLETIME, &integral, &toff);
            // Apply offset to timespec
            TS::AppyOffset(&tick, toff);
            // Monotonic sleep
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
            // Increment timespec by cycletime
            TS::Increment(tick, CYCLETIME);
        }
    }

    return 0;
}
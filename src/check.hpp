#pragma once

#include <filesystem>

//! @brief Asynchronously check the state of the EtherCAT network
//!
//! This function checks the state of the EtherCAT network and slaves. If a slave is not in the operational state,
//! it will attempt to recover it.
//!
//! @param fsm Pointer to the state machine
//! @param wkc Pointer to the working counter
//! @param expectedWKC The expected working counter value obtained after normal initialization
void check(Robot::FSM *fsm, int *wkc, int expectedWKC)
{
    int slave;
    bool operational = false;
    uint8_t currentgroup = 0;

    // Timing
    struct timespec tick;
    clock_gettime(CLOCK_MONOTONIC, &tick);
    int64_t serviceTime = CYCLETIME * 10;
    TS::Increment(tick, serviceTime);

    while (!fsm->shutdown)
    {
        if (EcatError)
        {
            fsm->eventLog.Debug(fmt::format("Error list: {}", ec_elist2string()));
        }
        // Check if all slaves are operational or if a slave requires a state change
        if (*wkc < expectedWKC || ec_group[currentgroup].docheckstate)
        {
            // One ore more slaves are not responding, inhibit run mode until it's cleared
            fsm->EtherCATFault = true;
            if (*wkc < expectedWKC && operational)
            {
                fsm->eventLog.EtherCAT(
                    fmt::format("WKC less than expected {} < {}, preventing further motion", *wkc, expectedWKC));
                operational = false;
            }

            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();

            // Check state of all slaves
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        fsm->eventLog.EtherCAT(fmt::format("Slave {} is in SAFE_OP + ERROR, attempting ack.", slave));
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        fsm->eventLog.EtherCAT(fmt::format("Slave {} is in SAFE_OP, change to OPERATIONAL.", slave));
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTSTATE))
                        {
                            ec_slave[slave].islost = FALSE;
                            fsm->eventLog.EtherCAT(fmt::format("Slave {} reconfigured", slave));
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            fsm->eventLog.EtherCAT(fmt::format("Slave {} lost", slave));
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTSTATE))
                        {
                            ec_slave[slave].islost = FALSE;
                            fsm->eventLog.EtherCAT(fmt::format("Slave {} recovered", slave));
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        fsm->eventLog.EtherCAT(fmt::format("Slave {} found", slave));
                    }
                }
            }
        }
        else if (*wkc >= expectedWKC && !operational)
        {
            fsm->eventLog.EtherCAT("Operational state reached for all slaves");
            operational = true;
            fsm->EtherCATFault = false;
        }

        // Monotonic sleep
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
        // Increment timespec by service time
        TS::Increment(tick, serviceTime);
    }
}

void systemCheck(Robot::FSM *fsm)
{
    // Attempt to open thermal zone file
    std::filesystem::path thermalZonePath = "/sys/class/thermal/thermal_zone1/temp";
    std::ifstream thermalZoneFile(thermalZonePath);
    if (!thermalZoneFile.is_open())
    {
        spdlog::error("Failed to open thermal zone file");
        return;
    }

    while (!fsm->shutdown)
    {
        thermalZoneFile.seekg(0);
        std::string temp;
        std::getline(thermalZoneFile, temp);
        double temperature = std::stoi(temp) / 1000.0;

        fsm->status.cpuTemperature = temperature;

        if (temperature > 80)
        {
            fsm->eventLog.Critical(fmt::format("CPU temperature too high: {} C", temperature));
            fsm->shutdown = true;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
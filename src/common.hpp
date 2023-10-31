#ifndef _COMMON_H
#define _COMMON_H

#include "delta.hpp"

#define SYNC0 2 * 1000 * 1000
#define CYCLETIME 2 * 1000 * 1000

static const int NSEC_PER_SECOND = 1e+9;
static const int USEC_PER_SECOND = 1e+6;

int64_t toff = 0;
int64_t integral = 0;
void ts_DCSync(int64_t reftime, int64_t cycletime, int64_t *offsettime)
{
    int64_t delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 500000) % cycletime;
    if (delta > (cycletime / 2))
    {
        delta = delta - cycletime;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }
    *offsettime = -(delta / 100) - (integral / 20);
}

void ts_AppyOffset(struct timespec *ts, int64 addtime)
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

void ts_Increment(struct timespec &tick, int nsec)
{
    tick.tv_nsec += nsec;
    while (tick.tv_nsec >= NSEC_PER_SECOND)
    {
        tick.tv_nsec -= NSEC_PER_SECOND;
        tick.tv_sec++;
    }
}

int wkc = 0;
static int moog_write8(uint16_t slave, uint16_t index, uint8_t subindex,
                       uint8_t value)
{
    int wkc;

    wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value,
                      EC_TIMEOUTRXM);
    return wkc;
}
static int moog_write16(uint16_t slave, uint16_t index, uint8_t subindex,
                        uint16_t value)
{
    int wkc;

    wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value,
                      EC_TIMEOUTRXM);
    return wkc;
}

static int Map_PDO_CMMT(uint16_t slave)
{
    wkc += moog_write8(slave, 0x60C2, 1, 1);
    wkc += moog_write8(slave, 0x60C2, 2, -3);

    const auto mapPDO = [&](const uint8_t slaveId, const uint16_t PDO_map, const uint32_t *data, const uint8_t dataSize, const uint32_t SM_map) -> int
    {
        int wkc;
        // Unmap previous registers, setting 0 in PDO_MAP subindex 0
        uint32_t zeroU32 = 0;
        wkc += ec_SDOwrite(slave, PDO_map, 0, FALSE, sizeof(zeroU32), &zeroU32, EC_TIMEOUTRXM);
        // Modify mapping, setting register address in PDO's subindexes from 0x1A00:01
        for (uint32_t i = 0; i < dataSize; i++)
        {
            uint8_t subIndex = static_cast<uint8_t>(i + 1);
            wkc += ec_SDOwrite(slave, SM_map, subIndex, FALSE, sizeof(data[i]), &data[i], EC_TIMEOUTRXM);
        }
        // Enable mapping by setting number of registers in PDO_MAP subindex 0
        wkc += ec_SDOwrite(slave, PDO_map, 0, FALSE, sizeof(dataSize), &dataSize, EC_TIMEOUTRXM);
        // Set PDO mapping to SM
        uint8_t zeroU8 = 0;
        // Unmap previous mappings, setting 0 in SM_MAP subindex 0
        wkc += ec_SDOwrite(slave, SM_map, 0, FALSE, sizeof(zeroU8), &zeroU8, EC_TIMEOUTRXM);
        // Write first mapping (PDO_map) address in SM_MAP subindex 1
        wkc += ec_SDOwrite(slave, SM_map, 1, FALSE, sizeof(PDO_map), &PDO_map, EC_TIMEOUTRXM);
        uint8_t pdoMapSize = 1;
        // Save mapping count in SM (here only one PDO_MAP)
        wkc += ec_SDOwrite(slave, SM_map, 0, FALSE, sizeof(pdoMapSize), &pdoMapSize, EC_TIMEOUTRXM);
        return wkc;
    };

    printf("MAP PDO ");
    wkc += mapPDO(slave, 0x1A00, tx_mapping, tx_mapping_count, 0x1C13);
    wkc += mapPDO(slave, 0x1600, rx_mapping, rx_mapping_count, 0x1C12);

    printf("slave_id: %x\n", slave);
    return 0;
}

#endif
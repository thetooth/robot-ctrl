#include "delta.hpp"

//! @brief Configure EtherCAT device
//!
//! This function configures the EtherCAT device and sets up the SDO parameters, sync manager and PDOs.
//! @param slave Slave ID
//! @return 0 on success, -1 on failure
int Delta::PO2SOconfig(uint16_t slave)
{
    auto wkc = 0;

    // Set interpolation values
    uint8_t interpolationPeriod = 2; // ms
    int8_t ratio = -3;               // See manual for details
    wkc += ec_SDOwrite(slave, 0x60C2, 1, FALSE, sizeof(interpolationPeriod), &interpolationPeriod, EC_TIMEOUTRXM);
    wkc += ec_SDOwrite(slave, 0x60C2, 2, FALSE, sizeof(ratio), &ratio, EC_TIMEOUTRXM);

    // Set homing mode
    uint8_t homingMode = 34;  // Got to z index
    uint32_t accel = 10 * 10; // Speed in rpm
    wkc += ec_SDOwrite(slave, 0x6098, 0, FALSE, sizeof(homingMode), &homingMode, EC_TIMEOUTRXM);
    wkc += ec_SDOwrite(slave, 0x6099, 1, FALSE, sizeof(accel), &accel, EC_TIMEOUTRXM);
    wkc += ec_SDOwrite(slave, 0x6099, 2, FALSE, sizeof(accel), &accel, EC_TIMEOUTRXM);

    const auto mapPDO = [&](const uint16_t PDO_map, const uint32_t *data, const uint8_t dataSize,
                            const uint32_t SM_map) -> int {
        int wkc = 0;
        // Unmap previous registers, setting 0 in PDO_MAP subindex 0
        uint32_t zeroU32 = 0;
        wkc += ec_SDOwrite(slave, PDO_map, 0, FALSE, sizeof(zeroU32), &zeroU32, EC_TIMEOUTRXM);
        // Modify mapping, setting register address in PDO's subindexes from 0x1A00:01
        for (uint32_t i = 0; i < dataSize; i++)
        {
            uint8_t subIndex = static_cast<uint8_t>(i + 1);
            wkc += ec_SDOwrite(slave, PDO_map, subIndex, FALSE, sizeof(data[i]), &data[i], EC_TIMEOUTRXM);
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

    spdlog::debug("MAP PDO slave {}", slave);
    wkc += mapPDO(0x1A00, tx_mapping, tx_mapping_count, 0x1C13);
    wkc += mapPDO(0x1600, rx_mapping, rx_mapping_count, 0x1C12);

    return wkc;
}

uint16_t Delta::PDO::getStatusWord() const
{
    return in->status_word;
}

int32_t Delta::PDO::getActualPosition() const
{
    return in->actual_position;
}

int32_t Delta::PDO::getActualVelocity() const
{
    return in->actual_velocity;
}

int16_t Delta::PDO::getActualTorque() const
{
    return in->actual_torque;
}

int32_t Delta::PDO::getFollowingError() const
{
    return in->following_error;
}

uint16_t Delta::PDO::getErrorCode() const
{
    return in->error_code;
}

uint32_t Delta::PDO::getDigitalInputs() const
{
    return in->digital_inputs;
}

bool Delta::PDO::getEmergencyStop() const
{
    return getDigitalInputs() & (1 << 16);
}

void Delta::PDO::setControlWord(uint16_t value)
{
    out->control_word = value;
}

void Delta::PDO::setTargetPosition(int32_t value)
{
    out->target_position = value;
}

void Delta::PDO::setTargetVelocity(int32_t value)
{
    out->target_velocity = value;
}

void Delta::PDO::setTargetTorque(int16_t value)
{
    out->target_torque = value;
}

void Delta::PDO::setDigitalOutputs(uint32_t value)
{
    out->digital_outputs = value;
}

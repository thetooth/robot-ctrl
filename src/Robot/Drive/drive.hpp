#ifndef FSM_DRIVE_HPP
#define FSM_DRIVE_HPP

#include "CAN/CoE.hpp"
#include "delta.hpp"
#include "ethercat.h"
#include "osal.h"
#include "oshw.h"

namespace Drive
{
    class Motor : public CANOpen::FSM
    {
      public:
        int slaveID;
        double gearRatio;
        bool fault;

        Delta::tx_t *InPDO;
        Delta::rx_t *OutPDO;

        Motor()
        {
            fault = true;
        }
        Motor(int id, double ratio) : slaveID(id), gearRatio(ratio), fault(false)
        {
            InPDO = (Delta::tx_t *)ec_slave[slaveID].inputs;
            OutPDO = (Delta::rx_t *)ec_slave[slaveID].outputs;
        }
        void update();
        bool move(double position);
        double getPosition();
        double getVelocity();
        int setModeOfOperation(CANOpen::control::mode value);
        int setHomingOffset(int32_t value);
    };
} // namespace Drive

#endif
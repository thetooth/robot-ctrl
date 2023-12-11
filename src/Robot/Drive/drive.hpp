#ifndef FSM_DRIVE_HPP
#define FSM_DRIVE_HPP

#include "CAN/CoE.hpp"
#include "delta.hpp"
#include "ethercat.h"
#include "nlohmann/json.hpp"
#include "osal.h"
#include "oshw.h"

namespace Drive
{
    namespace fmt = spdlog::fmt_lib;
    using json = nlohmann::json;

    class Motor : public CANOpen::FSM
    {
      public:
        int slaveID;
        double positionRatio, velocityRatio;
        double minPosition, maxPosition;
        bool fault;
        std::string lastFault = "I'm OK";

        Delta::tx_t *InPDO;
        Delta::rx_t *OutPDO;

        Motor()
        {
            fault = true;
        }
        Motor(int id, double positionRatio, double velocityRatio, double minimum, double maximum)
            : slaveID(id), positionRatio(positionRatio), velocityRatio(velocityRatio), minPosition(minimum),
              maxPosition(maximum), fault(false)
        {
            InPDO = (Delta::tx_t *)ec_slave[slaveID].inputs;
            OutPDO = (Delta::rx_t *)ec_slave[slaveID].outputs;
        }
        void update();
        bool move(double position);
        double getPosition() const;
        double getVelocity() const;
        double getTorque() const;
        double getFollowingError() const;
        int setModeOfOperation(CANOpen::control::mode value);
        int setHomingOffset(int32_t value);
        int setTorqueLimit(double value);
        int setFollowingWindow(double value);
        int faultReset();
    };

    void to_json(json &j, const Motor &m);
} // namespace Drive

#endif
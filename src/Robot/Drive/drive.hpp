#ifndef FSM_DRIVE_HPP
#define FSM_DRIVE_HPP

#include "CAN/CoE.hpp"
#include "ethercat.h"
#include "nlohmann/json.hpp"
#include "osal.h"
#include "oshw.h"
#include "pdo.hpp"

// import Leadshine;

namespace Drive
{
    namespace fmt = spdlog::fmt_lib;
    using json = nlohmann::json;

    class Motor : public CANOpen::FSM
    {
      public:
        int slaveID;
        PDO *pdo;
        double positionRatio, velocityRatio;
        double minPosition, maxPosition;
        bool fault;
        std::string lastFault = "OK";

        Motor()
        {
            fault = true;
        }
        Motor(int id, PDO *pdoImpl, double positionRatio, double velocityRatio, double minimum, double maximum)
            : slaveID(id), pdo(pdoImpl), positionRatio(positionRatio), velocityRatio(velocityRatio),
              minPosition(minimum), maxPosition(maximum), fault(false)
        {
        }
        void update();
        bool move(double position);
        double getPosition() const;
        double getVelocity() const;
        double getTorque() const;
        double getFollowingError() const;
        uint16_t getErrorCode() const;
        int setModeOfOperation(CANOpen::control::mode value);
        int setHomingOffset(int32_t value);
        int setTorqueLimit(double value);
        int setFollowingWindow(double value);
        int faultReset();
    };

    void to_json(json &j, const Motor &m);
} // namespace Drive

#endif

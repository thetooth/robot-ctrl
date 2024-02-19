#ifndef FSM_DRIVE_HPP
#define FSM_DRIVE_HPP

#include "CAN/CoE.hpp"
#include "ethercat.h"
#include "osal.h"
#include "oshw.h"
#include "pdo.hpp"

#include <deque>
#include <numeric>

namespace Drive
{
    namespace fmt = spdlog::fmt_lib;

    class Motor : public CANOpen::FSM
    {
      public:
        int slaveID;
        PDO *pdo;
        double positionRatio, velocityRatio;
        double minPosition, maxPosition;
        double torqueThreshold;
        bool fault;
        std::string lastFault = "OK";
        std::deque<double> torqueHistory;

        Motor()
        {
            fault = true;
        }
        Motor(int id, PDO *pdoImpl, double positionRatio, double velocityRatio, double minimum, double maximum)
            : slaveID(id), pdo(pdoImpl), positionRatio(positionRatio), velocityRatio(velocityRatio),
              minPosition(minimum), maxPosition(maximum), torqueThreshold(100), fault(false)
        {
        }
        void update();
        bool move(double position);
        double getPosition() const;
        double getVelocity() const;
        double getTorque() const;
        double getFollowingError() const;
        uint16_t getErrorCode() const;
        bool getEmergencyStop() const;
        int setModeOfOperation(CANOpen::control::mode value);
        int setHomingOffset(int32_t value);
        int setTorqueLimit(double value);
        int setTorqueThreshold(double value);
        int setFollowingWindow(double value);
        int faultReset();
    };

} // namespace Drive

#endif

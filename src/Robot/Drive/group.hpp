#ifndef FSM_DRIVE_GROUP_HPP
#define FSM_DRIVE_GROUP_HPP

#include "drive.hpp"

#include <vector>

namespace Drive
{
    class Group : Motor
    {
      public:
        std::vector<Motor *> drives;

        template <typename... Motor> Group(Motor &&...members)
        {
            drives = {members...};
        }

        void update();
        void setCommand(CANOpenCommand command);
        int setModeOfOperation(CANOpen::control::mode value);
        int setTorqueLimit(double value);
        int setFollowingWindow(double value);
        int faultReset();
    };

} // namespace Drive

#endif
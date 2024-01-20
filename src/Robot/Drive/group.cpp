#include "group.hpp"

void Drive::Group::update()
{
    for (auto &&drive : drives)
    {
        drive->update();
    }
}

void Drive::Group::setCommand(CANOpenCommand command)
{
    for (auto &&drive : drives)
    {
        drive->setCommand(command);
    }
}

int Drive::Group::setModeOfOperation(CANOpen::control::mode value)
{
    auto wkc = 0;
    for (auto &&drive : drives)
    {
        wkc += drive->setModeOfOperation(value);
    }
    return wkc;
}

int Drive::Group::setTorqueLimit(double value)
{
    auto wkc = 0;
    for (auto &&drive : drives)
    {
        wkc += drive->setTorqueLimit(value);
    }
    return wkc;
}

int Drive::Group::setTorqueThreshold(double value)
{
    auto wkc = 0;
    for (auto &&drive : drives)
    {
        wkc += drive->setTorqueThreshold(value);
    }
    return wkc;
}

int Drive::Group::setFollowingWindow(double value)
{
    auto wkc = 0;
    for (auto &&drive : drives)
    {
        wkc += drive->setFollowingWindow(value);
    }
    return wkc;
}

int Drive::Group::faultReset()
{
    auto wkc = 0;
    for (auto &&drive : drives)
    {
        wkc += drive->faultReset();
    }
    return wkc;
}

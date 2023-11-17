#include "fsm.hpp"

void Robot::to_json(json &j, const OTGStatus &p)
{
    j = json{{"result", p.result}};
}

void Robot::to_json(json &j, const Status &p)
{
    j = json{{"otg", p.otg}};
}
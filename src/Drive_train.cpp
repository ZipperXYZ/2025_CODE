#include "vex.h"

#include "Drive_train.h"
#include "MiniPID.h"

using namespace vex;

Drive_train::Drive_train(double p, double i, double d, double pt, double it, double dt) : DrivePID(p,i,d), TurnPID(pt,it,dt)
{
    DrivePID.setOutputLimits(-12,12);
    TurnPID.setOutputLimits(-12,12);
}

void Drive_train::Init()
{

}

int Drive_train::movefor()
{
    return 1;
}
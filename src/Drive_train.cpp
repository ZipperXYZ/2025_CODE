#include "vex.h"

#include "Drive_train.h"
#include "MiniPID.h"
#include "robot-config.h"

using namespace vex;

double ComputeError(double CurrentPoint,double EndPoint){
    return CurrentPoint - EndPoint;
}

double ComputeAverageDistance(){
    return (LeftDriveSmart.position(degrees) + RightDriveSmart.position(degrees))/2;
}

Drive_train::Drive_train(double p, double i, double d, double pt, double it, double dt) : DrivePID(p,i,d), TurnPID(pt,it,dt)
{
    DrivePID.setOutputLimits(-12,12);
    TurnPID.setOutputLimits(-12,12);
}

void Drive_train::Init()
{

}

int Drive_train::movefor(double Degree)
{
    double AverageDistance = ComputeAverageDistance();
    double EndPoint = AverageDistance + Degree;

    while (ComputeError(AverageDistance,Degree) > ErrorThreshold){
        
        AverageDistance = ComputeAverageDistance();
        
        double Output = DrivePID.getOutput(AverageDistance,EndPoint);

        LeftDriveSmart.spin(forward,Output,vex::volt);
        RightDriveSmart.spin(reverse,Output,vex::volt);
    }
    return 1;
}
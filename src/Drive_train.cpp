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

bool Drive_train::IsSettled(double Value){
    if (Value > ErrorThreshold) {
        return true;
    } else {
        return false;
    }
}

void Spin(double Output){
    LeftDriveSmart.spin(forward,Output,vex::volt);
    RightDriveSmart.spin(reverse,Output,vex::volt);
}

Drive_train::Drive_train(double p, double i, double d, double pt, double it, double dt) : DrivePID(p,i,d), TurnPID(pt,it,dt)
{
    DrivePID.setOutputLimits(-12,12);
    TurnPID.setOutputLimits(-12,12);
}

/// @brief 
void Drive_train::Init()
{

}

void Drive_train::Update(){

    InertialAverage = Inertial1.rotation(degrees);

    
}

int Drive_train::movefor(double Degree)
{
    double AverageDistance = ComputeAverageDistance();
    double EndPoint = AverageDistance + Degree;
    Brain.Screen.print("%f",ComputeError(AverageDistance,Degree));

    while (IsSettled(abs(ComputeError(AverageDistance,Degree)) == false)){
        
        AverageDistance = ComputeAverageDistance();
        
        double Output = DrivePID.getOutput(AverageDistance,EndPoint);
        Spin(Output);
    }
    
    return 1;
}
#include "vex.h"

#include "Drive_train.h"
#include "MiniPID.h"
#include "robot-config.h"
#include <cmath>
#include "util.h"

using namespace vex;

double ComputeError(double CurrentPoint,double EndPoint){
    return CurrentPoint - EndPoint;
}

double ComputeAverageDistance(){
    return (LeftDriveSmart.position(degrees) + RightDriveSmart.position(degrees))/2;
}

bool Drive_train::IsSettled(double Value){
    if (Value > ErrorThreshold) {
        return false;
    } else {
        return true;
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

// the main updte fonction containing odometry and other stuff

void Drive_train::Update(){

    InertialAverage = Inertial1.rotation(degrees);

    DistForwTrack = (FowardEncoder.position(degrees)/360) * (WheelDiameter * M_PI);
    DistSideTrack = (SideEncoder.position(degrees)/360) * (WheelDiameter * M_PI);

    RayonForw = (DistForwTrack/InertialAverage) + TF;
    RayonSide = (DistSideTrack/InertialAverage) + TS;
    
    if (InertialAverage == 0) {

        Y_Position = DistForwTrack + TF;
        X_Position = DistSideTrack + TS;

    } else {

        Y_Position += 2 * RayonForw * (sin(to_rad(InertialAverage/2)));
        X_Position += 2 * RayonSide * (sin(to_rad(InertialAverage/2)));

    }

}

int Drive_train::movefor(double Degree)
{
    double AverageDistance = ComputeAverageDistance();
    double EndPoint = AverageDistance + Degree;
    Brain.Screen.print("%f",std::abs(ComputeError(AverageDistance,Degree)));

    while (IsSettled(std::abs((ComputeError(AverageDistance,Degree))) == false)){
        
        AverageDistance = ComputeAverageDistance();
        
        double Output = DrivePID.getOutput(AverageDistance,EndPoint);
        Spin(Output);
    }
    
    return 1;
}
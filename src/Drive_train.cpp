#include "vex.h"

#include "Drive_train.h"
#include "MiniPID.h"
#include "robot-config.h"
#include <cmath>
//#include "util.h"

using namespace vex;

float ComputeError(float CurrentPoint,float EndPoint){
    return CurrentPoint - EndPoint;
}

float ComputeAverageDistance(){
    double RightSide = (RightDriveSmart.position(degrees)/360) * 3.25 * M_PI;
    double LeftSide = (LeftDriveSmart.position(degrees)/360) * 3.25 * M_PI;
    return (LeftSide + RightSide) / 2;
}

bool Drive_train::IsSettled(float Value){
    if (Value > ErrorThreshold) {
        return false;
    } else {
        return true;
    }
}

void Spin(float Output,float Output2){
    LeftDriveSmart.spin(forward,Output,vex::volt);
    RightDriveSmart.spin(forward,Output2,vex::volt);
}

Drive_train::Drive_train(float p, float i, float d, float pt, float it, float dt) : DrivePID(p,i,d), TurnPID(pt,it,dt)
{
    DrivePID.setOutputLimits(-12,12);
    TurnPID.setOutputLimits(-12,12);
}

/// @brief 
void Drive_train::Init()
{

}

// the main update fonction containing odometry and other stuff

void Drive_train::Update(){
    Brain.Screen.clearScreen();

    //to_rad(Inertial1.heading()); //AbsoluteAngle;

    InertialAverage = reduce_0_to_360(Inertial1.rotation(degrees));
    AbsoluteAngle_Rad = to_rad(InertialAverage);

    DistForwTrack = (FowardEncoder.position(degrees)/360) * (WheelDiameter * M_PI);
    DistSideTrack = -((SideEncoder.position(degrees)/360) * (WheelDiameter * M_PI));

    ChangeAngle_Rad = AbsoluteAngle_Rad - PrevAngle_Rad; // in rad
    ChangeDistForwTrack = DistForwTrack - PrevDistForwTrack; // in inches
    ChangeDistSideTrack = DistSideTrack - PrevDistSideTrack;
    
    PrevDistForwTrack = DistForwTrack;
    PrevDistSideTrack = DistSideTrack;
    PrevAngle_Rad = AbsoluteAngle_Rad;
    
    float Local_Polar_Angle;
    float Local_Polar_Radius;

    if (ChangeAngle_Rad == 0) {

        Local_Y_Position = ChangeDistSideTrack;
        Local_X_Position = ChangeDistSideTrack;
        Test += Local_Y_Position;

    } else {
        RayonForw = (ChangeDistForwTrack/ChangeAngle_Rad) + TF;
        RayonSide = (ChangeDistSideTrack/ChangeAngle_Rad) + TS;
        Local_Y_Position = ((ChangeDistForwTrack/ChangeAngle_Rad) + TF) * (2 * sin(ChangeAngle_Rad/2));
        Local_X_Position = ((ChangeDistSideTrack/ChangeAngle_Rad) + TS) * (2 * sin(ChangeAngle_Rad/2));
    }
    
    if (Local_X_Position == 0 && Local_Y_Position == 0){
        Local_Polar_Angle = 0;
        Local_Polar_Radius = 0;
    } else {
        Local_Polar_Angle = atan2(Local_Y_Position, Local_X_Position);
        Local_Polar_Radius = sqrt(pow(Local_X_Position,2) + pow(Local_Y_Position,2));
    }
    
    float global_polar_angle = Local_Polar_Angle - PrevAngle_Rad - (ChangeAngle_Rad/2);

    float AverageRotation = AbsoluteAngle_Rad + ChangeAngle_Rad / 2;

    float Delta_Global_X = Local_Polar_Radius *cos(global_polar_angle);//(Local_Y_Position * sin(AverageRotation)) - (Local_X_Position * cos(AverageRotation)); //(Local_Y_Position * cos(AverageRotation)) - (Local_X_Position * sin(AverageRotation));
    float Delta_Global_Y = Local_Polar_Radius *sin(global_polar_angle);//(Local_Y_Position * cos(AverageRotation)) + (Local_X_Position * sin(AverageRotation));//(Local_Y_Position * sin(AverageRotation)) - (Local_X_Position * cos(AverageRotation));

    X_Position += Delta_Global_X;
    Y_Position += Delta_Global_Y;

}

int Drive_train::turnto(float Angle){
    float PrevAngle = InertialAverage;
    float DeltaAngle = InertialAverage - PrevAngle;
    float CurrentAngle = InertialAverage;

    while (IsSettled(std::abs((ComputeError(InertialAverage,Angle))))){
        float Output = TurnPID.getOutput(InertialAverage,Angle); 
        Spin(Output,-Output); 
    }

    return 1;
}

int Drive_train::movefor(float Inches)
{
    float AverageDistance = Y_Position;
    float EndPoint = AverageDistance + Inches;
    Brain.Screen.print("%f",std::abs(ComputeError(AverageDistance,EndPoint)));

    while (IsSettled(std::abs((ComputeError(AverageDistance,EndPoint))) == false)){
        printf("error %f\n",AverageDistance);
        printf("---------------------\n");
        AverageDistance = Y_Position;
        
        float Output = DrivePID.getOutput(AverageDistance,EndPoint);
        Spin(Output,Output);
    }
    
    return 1;
}

int Drive_train::moveto(float X_Final,float Y_Final){
    
    float X_Delta = X_Final - X_Position;
    float Y_Delta = Y_Final - Y_Position;
    
    float Angle = to_deg(atan2(Y_Delta,X_Delta));
    float DistanceError = sqrt(pow(X_Delta,2) + pow(Y_Delta,2));
    float FinalDestination = sqrt(pow(X_Final,2) + pow(Y_Final,2));
    turnto(Angle);

    while (IsSettled(std::abs(ComputeError(DistanceError,FinalDestination)))){
        X_Delta = X_Final - X_Position;
        Y_Delta = Y_Final - Y_Position;
        DistanceError = sqrt(pow(X_Delta,2) + pow(Y_Delta,2));
        float Output = DrivePID.getOutput(DistanceError,FinalDestination);
        wait(20,msec);
    }

    return 1;
}
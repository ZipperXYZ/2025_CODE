#ifndef DRIVE_TRAIN_H
#define DRIVE_TRAIN_H
#pragma once
#include "math.h"
#include "stdio.h"
#include "time.h"
//#include "vex.h"
#include "vex_global.h"
#include "vex_imu.h"
#include "MiniPID.h"
#include "vex_motor.h"
#include "vex_task.h"

using namespace vex;

#include "MiniPID.h"

struct Drive_train
{
    // Initializer

    Drive_train(float p, float i, float d, float pt, float it, float dt); // "p" = le p de la pid pour avancer. "pt" = le p de la pid pour tourner  : DrivePID(p,i,d), TurnPID(pt,it,dt) {};

    // Function

    void Init();

    int movefor(float Degree);
    int moveto(float X_Final,float Y_Final);
    int turnto(float angle);

    void Update();

    // Variable

    float ErrorThreshold = 1;
    
    float p = 0;
    float i = 0;
    float d = 0;
    float pt = 0;
    float it = 0;
    float dt = 0;
    
    float InertialAverage;

    float Local_X_Position;
    float Local_Y_Position;
    float X_Position = 0;
    float Y_Position = 0;

    float TF = -2.25;
    float TS = -2.75;

    float DistForwTrack;
    float DistSideTrack;
    float PrevDistForwTrack = 0;
    float PrevDistSideTrack = 0;
    float ChangeDistForwTrack;
    float ChangeDistSideTrack;

    float RayonForw;
    float RayonSide;

    float AbsoluteAngle_Rad;
    float PrevAngle_Rad = 0;
    float ChangeAngle_Rad;

    float WheelDiameter = 2.75;

    float Test = 0;


    private:
        MiniPID DrivePID;
        MiniPID TurnPID;
    
        bool IsSettled(float Value);
};

#endif
void  vexcodeInit( void );
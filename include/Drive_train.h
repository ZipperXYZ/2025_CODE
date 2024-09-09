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

    Drive_train(double p, double i, double d, double pt, double it, double dt); // "p" = le p de la pid pour avancer. "pt" = le p de la pid pour tourner  : DrivePID(p,i,d), TurnPID(pt,it,dt) {};

    // Function

    void Init();
    int movefor(double Degree);

    void Update();

    // Variable

    double ErrorThreshold = 1;
    
    double p = 0;
    double i = 0;
    double d = 0;
    double pt = 0;
    double it = 0;
    double dt = 0;
    
    double InertialAverage;

    double X_Position;
    double Y_Position;


    private:
        MiniPID DrivePID;
        MiniPID TurnPID;
    
        bool IsSettled(double Value);
};

#endif
void  vexcodeInit( void );
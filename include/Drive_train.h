#include "MiniPID.h"

using namespace vex;

struct Drive_train
{
    // Initializer 

    Drive_train(double p,double i,double d,double pt,double it,double dt) : DrivePID(p,i,d), TurnPID(pt,it,dt) {}; // "p" = le p de la pid pour avancer. "pt" = le p de la pid pour tourner
    Drive_train() = default;

    // Function

    void Init();
    int movefor(double Degree);

    // Variable

    double ErrorThreshold = 1;


    private:
        MiniPID DrivePID;
        MiniPID TurnPID;
};


void  vexcodeInit( void );
#include "MiniPID.h"

using namespace vex;

struct Drive_train
{
    Drive_train(double p,double i,double d,double pt,double it,double dt) : DrivePID(p,i,d), TurnPID(pt,it,dt) {}; // "p" = le p de la pid pour avancer. "pt" = le p de la pid pour tourner
    Drive_train() = default;
    void Init();
    int movefor();


    private:
        MiniPID DrivePID;
        MiniPID TurnPID;
};


void  vexcodeInit( void );
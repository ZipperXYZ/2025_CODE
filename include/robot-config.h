#include "Drive_train.h"
using namespace vex;

extern brain Brain;

// VEXcode devices
extern smartdrive Drivetrain;
extern motor ClawMotor;
extern motor ArmMotor;

extern motor leftMotorA;
extern motor rightMotorA;

extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;
extern Drive_train DriveX;

extern inertial Inertial1;
extern inertial Inertial2;


/**
 * Used to initialize code/tasks/sdevices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
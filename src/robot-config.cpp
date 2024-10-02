#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftMotorA = motor(PORT5, ratio18_1, false);
motor leftMotorB = motor(PORT4, ratio18_1, true);
motor leftMotorC = motor(PORT3, ratio18_1, true); 
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB,leftMotorC);

motor rightMotorA = motor(PORT6, ratio18_1, true); 
motor rightMotorB = motor(PORT2, ratio18_1, false); 
motor rightMotorC = motor(PORT1, ratio18_1, false); 
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB,rightMotorC);

gyro TurnGyroSmart = gyro(Brain.ThreeWirePort.D);
smartdrive Drivetrain= smartdrive(LeftDriveSmart, RightDriveSmart, TurnGyroSmart, 319.19, 320, 165, mm, 1);
motor ArmMotor = motor(PORT8, ratio18_1, false);
Drive_train DriveX = Drive_train(0.01,0,0,1,0,0);

inertial Inertial1 = inertial(PORT9);
inertial Inertial2 = inertial(PORT16);

rotation FowardEncoder = rotation(PORT7);
rotation SideEncoder = rotation(PORT12);
controller Controller = controller();

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain gyro
  wait(200, msec);
  TurnGyroSmart.calibrate();
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the gyro calibration process to finish
  while (TurnGyroSmart.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}
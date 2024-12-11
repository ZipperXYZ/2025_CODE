#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftMotorA = motor(PORT5, ratio18_1, false); // false
motor leftMotorB = motor(PORT4, ratio18_1, true); // true
motor leftMotorC = motor(PORT13, ratio18_1, true);  // true
//motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB,leftMotorC);

motor rightMotorA = motor(PORT6, ratio18_1, true); // true
motor rightMotorB = motor(PORT2, ratio18_1, false); // false
motor rightMotorC = motor(PORT1, ratio18_1, false); // false
//motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB,rightMotorC);

//gyro TurnGyroSmart = gyro(Brain.ThreeWirePort.D);
//smartdrive Drivetrain= smartdrive(LeftDriveSmart, RightDriveSmart, TurnGyroSmart, 319.19, 320, 165, mm, 1);
//Drive_train DriveX = Drive_train(0.01,0,0,1,0,0);

//inertial Inertial2 = inertial(PORT9);
inertial Inertial2 = inertial(PORT10);

//rotation FowardEncoder = rotation(PORT7);
//rotation SideEncoder = rotation(PORT12);
controller Controller1 = controller();

/*motor Intake = motor(PORT15,ratio36_1,false);
motor MoteurBras = motor(PORT17,ratio36_1,false);

digital_out PneumaBras = digital_out(Brain.ThreeWirePort.C);
digital_out PneumaBut = digital_out(Brain.ThreeWirePort.D);*/

led intake = led(Brain.ThreeWirePort.D);
led Lift = led(Brain.ThreeWirePort.C);
led Clamp = led(Brain.ThreeWirePort.D);

motor MoteurBras = motor(PORT18,ratio6_1,false);
motor Intake_moteur = motor(PORT15,ratio36_1,false);
motor_group Intake = motor_group(Intake_moteur,MoteurBras);
optical ColorSensor = optical(PORT10);

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
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the gyro calibration process to finish

  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}
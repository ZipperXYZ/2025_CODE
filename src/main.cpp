/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Wed Sep 25 2019                                           */
/*    Description:  Clawbot (4-motor Drivetrain) - Template                   */
/*                                                                            */
/*    Name:                                                                   */
/*    Date                                                                    */
/*    Class:                                                                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    1, 10, 11, 20, D
// ClawMotor            motor         3               
// ArmMotor             motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "sys/_intsup.h"
#include "sys/_stdint.h"
#include "v5_apitypes.h"
#include "vex_drivetrain.h"
#include "vex_global.h"
#include "vex_motor.h"
#include "vex_motorgroup.h" 
#include "vex_task.h"
#include "vex_triport.h"
#include <ostream>
#include <string>

using namespace vex;

competition Competition;

//Drive chassis(TANK_TWO_ROTATION,RightDriveSmart,LeftDriveSmart,PORT9,3.25,0.75,360,7,); // ,0.75,7.-2.75,0,12,2.75,2.5)

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
TANK_TWO_ROTATION,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(rightMotorA,rightMotorB,rightMotorC),

//Right Motors:
motor_group(leftMotorA,leftMotorB,leftMotorC),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT9,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.75,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT7,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
-2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
0,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT12,

//Sideways tracker diameter (reverse to make the direction switch):
2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
2.5

);

bool AutoEnabled = false;
bool BrasUp = false;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

double TurnConstant = 1.5; // la distance en degrées que le moteur du bras va tourné
double TurnPerTurn = 6; // the number of turn it takes for the motor to do 1 full revolution of the conveyor

int SelectedAuto = -1; // bro si tu comprend pas ça ya des problème
int NbOfAuto = 2;

double Deadband = 20; // controller deadband

/////////////////////////////////////////////////////////////////////////////////////////////////////////////


// tout ce que les boutton font

void ButtonUpPressed(){
  MoteurBras.spinToPosition(MoteurBras.position(turns) + TurnConstant,turns,true);
}

void ButtonDownPressed() {
  MoteurBras.spinToPosition(round(MoteurBras.position(turns))/TurnPerTurn,turns);
}

void ButtonR1Released(){
  MoteurBras.stop();
  Intake.stop();
}

void ButtonR2Pressed(){
  double value = !PneumaBras.value();
  PneumaBras.set(value);
  BrasUp = value;

  
  if (BrasUp){
    Intake.stop();
  }
}

void ButtonL1Pressed(){
  PneumaBut.set(!PneumaBut.value());
}

void ButtonL2Released(){
  MoteurBras.stop();
}

int position_track_task(){
  chassis.position_track();
  return 1;
}

// la fonction update sert a updater toute les choses qui ont besoin d'être update en temps réelle.
// elle update (si le mode autonome est activé) l'odometry du robot (la position en X et Y)
// sinon elle va update les controlles du robot (les joystick et etc)

void update(){
  while (true) {
    if (Inertial1.isCalibrating() == false){
      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("X pos: %f",chassis.get_X_position());
      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("Y pos: %f",chassis.get_Y_position());
    }
    if (Competition.isEnabled()){
      if (Competition.isAutonomous()) {

      } else {

        bool ButtonBrasPressed = Controller.ButtonR1.pressing();
        bool ButtonSpinBrasPressed = Controller.ButtonL2.pressing();

        if (BrasUp && ButtonBrasPressed) {
          MoteurBras.spin(forward);
        } else if (ButtonBrasPressed) {
          MoteurBras.spin(forward);
          Intake.spin(forward);
        }

        if (ButtonSpinBrasPressed) {
          MoteurBras.spin(forward);
        }

        int LeftVelocity = Controller.Axis3.position(); //+ Controller.Axis1.position() * 2;
        int RightVelocity = Controller.Axis2.position();// - Controller.Axis1.position() * 2;
       
        if (abs(RightVelocity) < Deadband ) {
          RightVelocity = 0;
        }

        if (abs(LeftVelocity) < Deadband ) {
          LeftVelocity = 0;
        }
        
        LeftDriveSmart.setVelocity(LeftVelocity,percent);
        RightDriveSmart.setVelocity(RightVelocity,percent);
        RightDriveSmart.spin(forward);
        LeftDriveSmart.spin(forward);
      }
    }
    wait(20,msec);
  }
}

void BrainPressed(){

  SelectedAuto += 1;
  
  if (SelectedAuto > NbOfAuto) {
    SelectedAuto = 0;
  }

  switch (SelectedAuto){
  case 0:
    Brain.Screen.printAt(4,50,"Selected Auto1");
    break;
  case 1:
    Brain.Screen.printAt(4,50,"Selected Auto2");
    break;
  default:
    break;
  }
}

void Autonomous(){

}

void PreAuto(){
  chassis.set_coordinates(0,0,0);
}

// la fonction main qui gère: le reset des encodeur, les 2 fonction de competition et la tache d'update

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  FowardEncoder.resetPosition();
  SideEncoder.resetPosition();
  Competition.autonomous(Autonomous); // les 2 template de compétition
  Competition.drivercontrol(update);

  // les controle

  Controller.ButtonR2.pressed(ButtonR2Pressed);
  Controller.ButtonL1.pressed(ButtonL1Pressed);
  Controller.ButtonUp.pressed(ButtonUpPressed);
  Controller.ButtonDown.pressed(ButtonDownPressed);
  Brain.Screen.pressed(BrainPressed);

  Controller.ButtonR1.released(ButtonR1Released);
  Controller.ButtonL2.released(ButtonL2Released);

  // odometry stuff and PID

  chassis.set_drive_constants(10,1,0.01,0.01,3);
  chassis.set_turn_constants(7,0.14,0.005,1.25,9);
  chassis.set_heading_constants(7,0.5,0.01,1.5,20);
  chassis.set_drive_constants(10,0.55,0.0125,2.5,3);
  chassis.set_drive_exit_conditions(0.5,200,4000);

  chassis.drive_timeout = 3000;

  chassis.DriveL.resetPosition();
  chassis.DriveR.resetPosition();

  // speed

  MoteurBras.setVelocity(100,percent);
  Intake.setVelocity(100,percent);

  //task upd(update);
  task updo(position_track_task);
  while (chassis.Gyro.isCalibrating())
  {
    wait(20,msec);
  }
  //DriveX.movefor(24);
}

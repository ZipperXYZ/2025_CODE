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
#include <autons.h>

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
motor_group(leftMotorA,leftMotorB,leftMotorC),

//Right Motors:
motor_group(rightMotorA,rightMotorB,rightMotorC),

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
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-0.25,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT12,

//Sideways tracker diameter (reverse to make the direction switch):
2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
2.25 // 2.5

);

int current_auton_selection = 0;
bool auto_started = false;


bool AutoEnabled = false;
bool BrasUp = false;
bool IsSkill = false;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

double TurnConstant = 1.5; // la distance en degrées que le moteur du bras va tourné
double TurnPerTurn = 6; // the number of turn it takes for the motor to do 1 full revolution of the conveyor

int SelectedAuto = 0; // bro si tu comprend pas ça ya des problème
int NbOfAuto = 8;

double Deadband = 20; // controller deadband 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// tout ce que les boutton font

// le bras tourne de 1 position de crochet

void ButtonUpPressed(){
  IntakeUntilDisk();
  //MoteurBras.spinToPosition(MoteurBras.position(turns) + TurnConstant,turns,true);
}

// stop les moteur du bras et de l'intake quand le boutton r1 est relaché

void ButtonR1Released(){
  MoteurBras.stop();
  Intake.stop();
}

// active la pneuma du bras et désactive le moteur de l'intake si le bras est levé

void ButtonL1Pressed(){
  double value = !Lift.value();
  Lift.set(value);
  BrasUp = value;

  
  if (BrasUp){
    Intake.stop();
  }
}

void ButtonXReleased(){
  Intake_moteur.stop();
}

// active la petite pince pour les but

void ButtonR2Pressed(){
  Clamp.off();
 // Clamp.set(!Clamp.value());
}

// stop le moteur du bras quand le boutton n'est pas pressé

void ButtonL2Released(){
  Intake.stop();
}

void ButtonDownReleased(){
  Intake_moteur.stop();
}

void ButtonBPressed(){
  Clamp.on();
}

int position_track_task(){
  chassis.position_track();
  return 1;
}

int printPosition(){
   while (true) {
      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("X pos: %f",chassis.get_X_position());
      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("Y pos: %f",chassis.get_Y_position());
   }
   return 1;
}

// la fonction update sert a updater toute les choses qui ont besoin d'être update en temps réelle.
// elle update (si le mode autonome est activé) l'odometry du robot (la position en X et Y)
// sinon elle va update les controlles du robot (les joystick et etc)

void update(){
  chassis.DriveL.setStopping(coast);
  chassis.DriveR.setStopping(coast);
   while (true) {
    if (Competition.isEnabled()){
      if (Competition.isAutonomous()) {

      } else {

        bool ButtonBrasPressed = Controller1.ButtonR1.pressing();
        bool ButtonSpinBrasPressed = Controller1.ButtonL2.pressing();
        bool ButtonXPressed = Controller1.ButtonX.pressing();
        bool ButtonDownPressed = Controller1.ButtonDown.pressing();

        // tourne l'intake si le bras est lever sinon, tourne les deux si le boutton est pressé

        if (BrasUp && ButtonBrasPressed) {
          MoteurBras.spin(forward);
        } else if (ButtonBrasPressed) {
          MoteurBras.spin(forward);
          Intake.spin(forward);
        }

        if (ButtonSpinBrasPressed) {
          Intake.spin(reverse);
        }

        if (ButtonXPressed){
          Intake_moteur.spin(forward);
        }

        if (ButtonDownPressed){
          Intake_moteur.spin(forward);
        }

        int LeftVelocity = Controller1.Axis3.position(); //+ Controller.Axis1.position() * 2;
        int RightVelocity = Controller1.Axis2.position();// - Controller.Axis1.position() * 2;
       
        if (abs(RightVelocity) < Deadband ) {
          RightVelocity = 0;
        }

        if (abs(LeftVelocity) < Deadband ) {
          LeftVelocity = 0;
        }

        chassis.control_tank();
      }
    }
    wait(20,msec);
  }
}

void BrainPressed(){

  SelectedAuto += 1;
  Brain.Screen.clearScreen();
  if (SelectedAuto > NbOfAuto) {
    SelectedAuto = 0;
  }
  switch (SelectedAuto){
  case 0:
    Brain.Screen.printAt(4,90,"Selected: rouge gauche win point");
    break;
  case 1:
    Brain.Screen.printAt(4,90,"Selected: rouge droite rush goal");
    break;
  case 2:
    Brain.Screen.printAt(4,90,"Selected: bleu droite win point");
    break; 
  case 3:
    Brain.Screen.printAt(4,90,"Selected: bleu gauche rush goal");
    break;
  case 4:
    Brain.Screen.printAt(4,90,"Selected: Skill");
    break;
  case 5:
    Brain.Screen.printAt(4,90,"Selected: rouge droite final");
    break;
  case 6:
    Brain.Screen.printAt(4,90,"Selected: rouge gorge final");
    break;
  case 7:
    Brain.Screen.printAt(4,90,"Selected: un geai bleu communiste qui est en final (bleu gauche final)");
    break;
  case 8:
    Brain.Screen.printAt(4,90,"Selected: bleu droite final");
    break;
  default:
    break;
  }
}

void Autonomous(){
  switch (SelectedAuto){
  case 0:
    Rouge_Gauche_WP();
    break;
  case 1:
    Rouge_Droit_Rush_Goal();
    break;
  case 2:
    Bleu_Droite_WP();
  case 3:
    Bleu_Gauche_Rush_Goal();
    break;
  case 4:
    Skill();
    break;
  case 5:
    RougeDroiteFinal();
    break;
  case 6:
    RougeGaucheFinal();
    break;
  case 7:
    BleuGaucheFinal();
    break;
  case 8:
    BleuDroiteFinal();
    break;
  default:
    break;
  }
}

void PreAuto(){
  vexcodeInit();
  default_constants();
  chassis.DriveL.resetPosition();
  chassis.DriveR.resetPosition();
  chassis.R_SidewaysTracker.resetPosition();
  chassis.R_ForwardTracker.resetPosition();
  Clamp.off();
  //chassis.set_coordinates(0,0,0);
}

// la fonction main qui gère: le reset des encodeur, les 2 fonction de competition et la tache d'update

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  Competition.autonomous(Autonomous); // les 2 template de compétition
  Competition.drivercontrol(update);

  // les controle

  Controller1.ButtonR2.pressed(ButtonR2Pressed);
  Controller1.ButtonL1.pressed(ButtonL1Pressed);
  Controller1.ButtonUp.pressed(ButtonUpPressed);
  Controller1.ButtonB.pressed(ButtonBPressed);
  Brain.Screen.pressed(BrainPressed);

  Controller1.ButtonR1.released(ButtonR1Released);
  Controller1.ButtonL2.released(ButtonL2Released);
  Controller1.ButtonDown.released(ButtonDownReleased);
  Controller1.ButtonX.released(ButtonXReleased);
  ColorSensor.setLight(vex::ledState::on);
  ColorSensor.setLightPower(100,percent);
  // odometry stuff and PID

  /*chassis.set_drive_constants(10,1,0.01,0.01,3);
  chassis.set_turn_constants(7,0.14,0.005,1.25,9);
  chassis.set_heading_constants(7,0.5,0.01,1.5,20);
  chassis.set_drive_constants(10,0.55,0.0125,2.5,3);
  chassis.set_drive_exit_conditions(0.5,200,4000);
  chassis.set_turn_exit_conditions(2,300,4000);*/

 // chassis.drive_timeout = 3000;

  // speed

  MoteurBras.setVelocity(100,percent);
  Intake_moteur.setVelocity(100,percent);


  task print(printPosition);
  PreAuto();
  task updo(position_track_task);
  while (true) {
    wait(100, msec);
  }
  //DriveX.movefor(24);
}

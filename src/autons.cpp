#include "vex.h"
#include <autons.h>

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  
  /*chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);*/

  chassis.set_drive_constants(10,1,0.01,0.01,1); // 3
  chassis.set_turn_constants(7,0.14,0.005,1.25,30);
  chassis.set_heading_constants(7,0.5,0.01,5.3,20); //5 // 2 // 1.5
  chassis.set_drive_constants(10,0.55,0.0125,2.5,5);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(2.6, 200, 5000); // 5000
  chassis.set_turn_exit_conditions(3, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
* Sets constants to be more effective for odom movements.
* For functions like drive_to_point(), it's often better to have
* a slower max_voltage and greater settle_error than you would otherwise.
*/

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 4;
  chassis.drive_max_voltage = 7;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

void Bleu_Droite_WP(){
  Clamp.off();
  chassis.drive_distance(-12.9, 0, 7, 5);
  chassis.turn_to_angle(270, 6);
  chassis.drive_distance(-6, 270, 7, 5);
  Intake.setVelocity(50, percent);
  Intake.spinFor(forward, 1, seconds);
  Intake.setVelocity(100, percent);
  chassis.drive_distance(27, 300, 10, 8);
  Clamp.on();
  chassis.turn_to_angle(140, 7);
  chassis.drive_distance(-12, 140, 6, 8);
  Clamp.off();
  chassis.turn_to_angle(340, 6);
  Intake.spin(forward);
  chassis.drive_distance(26, 310, 0010, 6);
  chassis.drive_distance(-8, 340, 10, 6);
  chassis.drive_distance(17, 340, 5, 6);
  chassis.turn_to_angle(85, 8);
  chassis.drive_distance(20, 90, 10, 8);
  chassis.turn_to_angle(210, 6);
  chassis.drive_distance(40, 205, 12, 8);
  Lift.on();
  Clamp.on();
  Intake.stop();
}

// 4 Ring + toucher échelle (6 pts) Bleu Côté Droit

void Bleu_Droite(){
  chassis.drive_distance(-30, 17, 9, 8);
  chassis.drive_distance(-3, 17, 9, 8);
  Clamp.off();
  Intake.setVelocity(100, percent);
  chassis.turn_to_angle(230, 6);
  Intake.spin(forward);
  chassis.drive_distance(18, 220, 10, 6);
  chassis.drive_distance(-12, 237, 10, 6);
  chassis.drive_distance(15, 236, 5, 6);
  chassis.turn_to_angle(0, 6);
  chassis.drive_distance(15, 0, 6, 6);
  Lift.off();
  chassis.turn_to_angle(130, 6);
  chassis.drive_distance(22, 250, 10, 6);
  wait(1, seconds);
  Clamp.on();
  Lift.on();
  wait(1, seconds);
  Intake.stop();
}

void testauto(){
  chassis.drive_to_pose(24,24,90,0.5,1,5); // setback 1
  //chassis.drive_to_point(24,48);
  //chassis.drive_to_point(0,72);
  //chassis.drive_to_point(48,120);
 //chassis.turn_to_angle(20);
  //chassis.drive_distance(24,0,12,12);
}



// AWP Solo (7 pts) Bleu Côté Gauche

void Bleu_Gauche_WP(){

}

// 2 top ring + échelles (6 pts) Bleu Côté Gauche
void Bleu_Gauche(){

}

// AWP Solo (7 pts) Rouge Côté Gauche

void Rouge_Gauche_WP(){
  Clamp.off();
  chassis.drive_distance(-9.9, 0, 7, 5);
  chassis.turn_to_angle(90, 6);
  chassis.drive_distance(-6, 90, 7, 5);
  Intake.setVelocity(50, percent);
  Intake.spinFor(forward, 1, seconds);
  Intake.setVelocity(100, percent);
  chassis.drive_distance(30, 60, 10, 8);
  Clamp.on();
  chassis.turn_to_angle(235, 7);
  chassis.drive_distance(-15, 230, 10, 8);
  Clamp.off();
  chassis.drive_distance(-5, 230, 10, 8);
  chassis.turn_to_angle(20, 6);
  Intake.spin(forward);
  chassis.drive_distance(17.5, 55, 10, 8);
  chassis.drive_distance(-14.5, 40, 10, 8);
  chassis.turn_to_angle(340, 8);
  chassis.drive_distance(20, 340, 10, 8);
  chassis.turn_to_angle(160, 6);
  chassis.drive_distance(38, 160, 12, 8);
  Lift.on();
  Clamp.on();
  Intake.stop();
  
}

// 4 ring + échelle (6 pts) Rouge Côté Gauche

void Rouge_Gauche(){
  chassis.drive_distance(-30.5, 343, 9, 8);
  Clamp.off();
  Intake.setVelocity(100, percent);
  chassis.turn_to_angle(130, 6);
  Intake.spin(forward);
  chassis.drive_distance(17, 140, 10, 6);
  chassis.drive_distance(-10, 123, 10, 6);
  chassis.drive_distance(16.6, 124, 5, 6);
  chassis.turn_to_angle(0, 6);
  chassis.drive_distance(15, 0, 6, 6);
  Lift.off();
  chassis.turn_to_angle(250, 6);
  chassis.drive_distance(22, 250, 10, 6);
  wait(1, seconds);
  Clamp.on();
  Lift.on();
  wait(1, seconds);
  Intake.stop();
}

// AWP Solo (7 pts) Rouge Côté Droit

void Rouge_Droite_WP(){
  
}

// 2 top ring + échelle (6 pts) Rouge Côté Droit

void Rouge_Droite(){
  chassis.drive_distance(-9.9, 0, 7, 5);
}



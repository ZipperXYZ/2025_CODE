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

  chassis.set_drive_constants(10,1,0.1,0.01,3); // i = 1/0.1/0.005/0.01 p = /0.6/ 0.8 / 1
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
  chassis.AngleReversed = true;
  Clamp.off();
  chassis.drive_settle_time = 150;
  chassis.turn_settle_time = 150;
  chassis.drive_distance(-9.9, 0, 12, 5);
  chassis.turn_to_angle(90, 12);
  chassis.drive_distance(-6, 90, 12, 5);
  Intake.setVelocity(50, percent);
  Intake.spinFor(forward, 1, seconds);
  Intake.setVelocity(100, percent);

  chassis.drive_distance(25, 60, 12, 8);
  Clamp.on();
  chassis.turn_to_angle(230, 12);
  chassis.drive_distance(-20, 230, 12, 8);
  Clamp.off();
  chassis.turn_to_angle(0,12);
  chassis.drive_to_pose(-36,24,0,0.5,1,0);
  Intake.spin(forward);
  chassis.drive_distance(10,0,12,5);
  chassis.turn_to_angle(90);
  chassis.drive_distance(10, 90, 12, 8);
  chassis.drive_to_pose(-44,0,90,0.5,1,0);
  wait(3,seconds);
  Lift.on();
  Clamp.on();
  Intake.stop();
  wait(20,seconds);
}

void testauto(){
  //chassis.turn_to_angle(90,12);
 // chassis.drive_distance(24,0,12,12);
  chassis.set_heading_constants(7,0.5,0.005,6.6,10); //5 // 2 // 1.5 D: 6.6 I: 0.005
  chassis.set_coordinates(-56,52,90);
  task disk3(IntakeUntilDisk);
  chassis.drive_to_pose(-9,84,45,0.5,1,0,12,12);
  chassis.turn_to_angle(220);
  chassis.drive_distance(-16,220);
  Clamp.off();
  Intake.spin(forward);
  wait(0.3,seconds);

  // va chercher les 3 disques rouge du coins avec les disques bleu sur le top

  chassis.drive_to_pose(-50,78,270,0.5,1,0,12,12);
  chassis.drive_distance(-13,0);
  chassis.turn_to_angle(0);
  chassis.drive_distance(24,0);
  Intake.stop();
  chassis.drive_distance(15,75);
  Intake.spin(forward);
  chassis.drive_distance(-20,0);
  chassis.drive_to_pose(chassis.get_X_position() + 6,chassis.get_Y_position() + 20,0,0.5,1,0,12,12);
  chassis.drive_distance(-6,0);
  chassis.turn_to_angle(270);
  chassis.drive_distance(12,270);
  chassis.turn_to_angle(160);
  Clamp.on();
  chassis.drive_distance(-14,160);
  Intake.stop();
  chassis.drive_distance(14,160);

  // va chercher le disque random et le mets dans le alliance stake bleu
  task disk4(IntakeUntilDisk);
  chassis.drive_to_pose(24,68,180,0.5,1,0,12,12);
  Clamp.off();
  chassis.drive_to_pose(0,105,180,0.5,1,0,12,12);
  Intake.spin(forward);


  //chassis.turn_to_angle(270);s
  //chassis.drive_to_pose(9,11.5,90,0.5,1,0); // setback 1 // drive min /12/0/ 5
  //chassis.turn_to_angle(0);ss
  //chassis.drive_to_pose(21,38,0,0.5,1,0,12,12);
  //chassis.drive_to_pose(0,48,90,0.5,1,0); // setback 1 // drive min /12/0/ 5
  //chassis.drive_to_pose(0,72,0,0.5,1,0);
  //chassis.drive_to_pose(24,96,180,0.5,1,0);
  //chassis.drive_to_point(24,48);
  //chassis.drive_to_point(0,72);
  //chassis.drive_to_point(48,120);
 //chassis.turn_to_angle(20);
  //chassis.drive_distance(24s,0ss,12,12);
  /*chassis.set_coordinates(56,55.5,270);
  chassis.drive_distance(5,270);
  chassis.turn_to_angle(90);
  chassis.drive_settle_error = 0.5;
  chassis.drive_to_pose(-14,13.5,90,-0.5,14,0,12,12);
  chassis.drive_distance(-7,90);
  Clamp.off();*/
}


void Skill(){
  //chassis.set_heading_constants(7,0.5,0.05,5.5,60);
  chassis.set_heading_constants(7,0.5,0.005,6.6,10);
  chassis.drive_settle_time = 150;
  chassis.turn_settle_time = 150;
  Clamp.off();
  chassis.drive_distance(-4,0);
  Intake.spin(forward);
  wait(0.5,seconds);
  Clamp.on();
  chassis.drive_distance(4,0);

  // va chercher le premier but

  chassis.turn_to_angle(270,12);
  chassis.drive_to_pose(6.5,9.5,270,0.01,0,0,12,12);
  chassis.drive_distance(-9,270);
  Clamp.off();
  wait(0.3,seconds);

  // va chercher le premier disque et le deuxieme disque
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 0;
  chassis.turn_to_angle(0);
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 150;
  chassis.drive_to_pose(21,34,0,0.5,1,9,12,12);

  chassis.turn_to_angle(90);
  chassis.drive_distance(27,90);

  // va chercher les 3 disques dans le coin 

  chassis.drive_to_pose(62,18,180,0.5,4,9,12,12);
  chassis.turn_to_angle(270);
  chassis.drive_distance(12,270);
  wait(0.3,seconds);
  chassis.drive_distance(-7,270);
  chassis.turn_to_angle(205);
  chassis.drive_distance(14,205);

  // va dans le coin et dépose le but
  chassis.drive_distance(-5,310);
  Clamp.on();
  chassis.drive_distance(-6,310);
  Intake.stop();

  // prendre le ring et le mettre sur le top truc
  chassis.drive_settle_time = 400;
  chassis.drive_settle_error = 0.1;
  wait(0.2,seconds);
  task Disk(IntakeUntilDisk);
  chassis.drive_to_pose(57,56,0,0.5,4,0,12,12);
  printf("pos X: %f pos y: %f",chassis.get_X_position(),chassis.get_Y_position());
  chassis.drive_settle_error = 2.6;
  wait(0.3,seconds);
  Clamp.off();
  chassis.turn_to_angle(270);
  Lift.off();
  chassis.drive_distance(-9,270,12,12,2.6,250,400);
  chassis.drive_timeout = 3000;
  Intake.stop();
  wait(1,seconds);
  Intake.spin(forward);
  wait(1,seconds);
  Intake.stop();
  Lift.on();

  // va vers le point 0,0 pour la deuxieme séquence laurtre bord

  chassis.drive_distance(7,0);
  Clamp.on(); 
  chassis.turn_to_angle(90);
  chassis.drive_settle_error = 0.5;
  chassis.drive_to_pose(-11,13.5,90,-0.5,16,0,12,12);
  chassis.drive_distance(-11,90);
  Clamp.off();
  wait(0.5,seconds);

  // legit la meme chose que au début mais a l'inverse

  //////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////

  // va chercher le premier disque et le deuxieme disque
  Intake.spin(forward);
  chassis.AngleReversed = true;
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 0;
  chassis.turn_to_angle(0);
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 150;
  chassis.drive_distance(22,0);
  chassis.drive_distance(-2,0);

  //chassis.drive_to_pose(-21,34,0,0.5,1,5,12,12);

  chassis.turn_to_angle(90);
  chassis.drive_distance(27,90);

  // va chercher les 3 disques dans le coin 

  chassis.drive_to_pose(-62,18,180,0.5,4,6,12,12);
  chassis.turn_to_angle(270);
  chassis.drive_distance(12,270);
  wait(0.3,seconds);
  chassis.drive_distance(-7,270);
  chassis.turn_to_angle(205);
  chassis.drive_distance(14,205);

  // va dans le coin et dépose le but
  chassis.drive_distance(-5,310);
  Clamp.on();
  chassis.drive_distance(-6,310);
  Intake.stop();

  // prendre le ring et le mettre sur le top truc
  chassis.drive_settle_time = 250;
  chassis.drive_settle_error = 0.1;
  task Disk2(IntakeUntilDisk);
  chassis.drive_to_pose(-56,52,0,0.5,4,0,9,9);
  chassis.drive_settle_error = 2.6;
  wait(0.3,seconds);
  Clamp.off();
  chassis.turn_to_angle(270);
  Lift.off();
  chassis.drive_distance(-10,270);
  Intake.stop();
  wait(1,seconds);
  Intake.spin(forward);
  wait(1,seconds);
  Intake.stop();
  Lift.on();
  chassis.drive_distance(8,90);
  chassis.turn_to_angle(90);

  // va chercher le beaux disque a raph et va prendre lautre but weird

  chassis.drive_to_pose(-20,74,90,0.5,4,0,12,10);
  chassis.turn_to_angle(270);
  chassis.drive_to_pose(0,90,270,0.5,-6,0,12,10);  //chassis.drive_to_pose(21,13,270,0.5,4,0,12,10);
}

// AWP Solo (7 pts) Rouge Côté Gauche

void Rouge_Gauche_WP(){
  Clamp.off();
  chassis.drive_settle_time = 150;
  chassis.turn_settle_time = 150;
  chassis.drive_distance(-9.9, 0, 12, 5);
  chassis.turn_to_angle(90, 12);
  chassis.drive_distance(-6, 90, 12, 5);
  Intake.setVelocity(50, percent);
  Intake.spinFor(forward, 1, seconds);
  Intake.setVelocity(100, percent);

  chassis.drive_distance(25, 60, 12, 8);
  Clamp.on();
  chassis.turn_to_angle(230, 12);
  chassis.drive_distance(-20, 230, 12, 8);
  Clamp.off();
  chassis.turn_to_angle(0,12);
  chassis.drive_to_pose(36,24,0,0.5,1,0);
  Intake.spin(forward);
  chassis.drive_distance(10,0,12,5);
  chassis.turn_to_angle(90);
  chassis.drive_distance(10, 90, 12, 8);
  chassis.drive_to_pose(44,0,90,0.5,1,0);
  wait(3,seconds);
  Lift.on();
  Clamp.on();
  Intake.stop();
  wait(20,seconds);
  
}

// 2 top ring + échelle (6 pts) Rouge Côté Droit

void Rouge_Droite(){
  chassis.drive_distance(-9.9, 0, 7, 5);
}

void Bleu_Gauche_Rush_Goal(){
  //chassis.X_Reversed = true;
  //chassis.turn_to_angle(330);
  //chassis.drive_to_pose(0,-10,150,0.5,0,0,12,10);
  chassis.drive_to_pose(9,-42,150,0.5,-4,0,12,10);
  chassis.drive_settle_time = 100;
  Clamp.off();
  chassis.drive_settle_time = 250;
  Intake.spin(forward);
  chassis.drive_distance(6,35);
  //chassis.turn_to_angle(30,9);
  //chassis.drive_distance(12,30);
  
  chassis.turn_to_angle(280);
  wait(0.3,seconds);
  Clamp.on();
  chassis.drive_distance(5,280);
  chassis.turn_to_angle(90);
  chassis.drive_to_pose(-10,-24,270,0.5,-6,0,12,10);
  //chassis.drive_distance(-22,90);
  Clamp.off();
  chassis.drive_timeout = 1500;
  chassis.drive_to_pose(26,0,90,0.5,9,0,12,10);
  chassis.turn_to_angle(50);
  chassis.drive_distance(17,50);
  wait(0.6,seconds);
  chassis.turn_to_angle(270);
  //chassis.drive_distance(-40,chassis.get_absolute_heading());
  chassis.drive_to_pose(-20,-43,180,0.5,-6,0,12,10);
  Intake.stop();

}

void Rouge_Droit_Rush_Goal(){
  chassis.AngleReversed = true;
  //chassis.X_Reversed = true;
  //chassis.turn_to_angle(330);
  //chassis.drive_to_pose(0,-10,150,0.5,0,0,12,10);
  chassis.drive_to_pose(-9,-42,150,0.5,-4,0,12,10);
  chassis.drive_settle_time = 100;
  Clamp.off();
  chassis.drive_settle_time = 250;
  Intake.spin(forward);
  chassis.drive_distance(4,chassis.get_absolute_heading());
  //chassis.turn_to_angle(30,9);
  //chassis.drive_distance(12,30);
  
  chassis.turn_to_angle(280);
  wait(0.3,seconds);
  Clamp.on();
  chassis.drive_distance(5,280);
  chassis.turn_to_angle(90);
  chassis.drive_to_pose(10,-24,270,0.5,-6,0,12,10);
  //chassis.drive_distance(-22,90);
  Clamp.off();
  chassis.drive_timeout = 1500;
  chassis.drive_to_pose(-26,0,90,0.5,9,0,12,10);
  chassis.turn_to_angle(50);
  chassis.drive_distance(17,50);
  wait(0.6,seconds);
  chassis.turn_to_angle(270);
  //chassis.drive_distance(-40,chassis.get_absolute_heading());



  //Intake.stop();
}



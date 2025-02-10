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

int IntakeButWait(void *arg){
  double *timewait = (double *)arg;
  wait(*timewait,seconds);
  Intake.spin(forward);
  return 1;
}

void Bleu_Droite_WP(){
  chassis.AngleReversed = true;
  Clamp.off();
  chassis.drive_settle_time = 150;
  chassis.turn_settle_time = 150;
  chassis.drive_distance(-9.9, 0, 12, 5);
  chassis.turn_to_angle(90, 12);
  chassis.drive_timeout = 600;
  chassis.drive_distance(-9, 90, 12, 5);
  chassis.drive_timeout = 2000;
  Intake.setVelocity(100, percent);
  Intake.spinFor(forward, 1, seconds);
  Intake.setVelocity(100, percent);

  chassis.drive_distance(18, 60, 12, 8);
  Clamp.on();
  chassis.turn_to_angle(230, 12);
  chassis.drive_distance(-17, 230, 12, 8);
  Clamp.off();
  chassis.turn_to_angle(0,12);
  chassis.drive_to_pose(-36,17,0,0.5,1,6);
  Intake.spin(forward);
  chassis.drive_distance(10,0,12,5);
  chassis.turn_to_angle(90);
  chassis.drive_distance(10, 90, 12, 8);
  chassis.drive_to_pose(-44,4,90,0.5,1,0);
  wait(3,seconds);
  Lift.on();
  Clamp.on();
  Intake.stop();
  wait(20,seconds);
}

void testauto(){
  //chassis.turn_to_angle(90,12);
 // chassis.drive_distance(24,0,12,12);
  // chassis.set_drive_constants(0,1,0,10,3);
 // chassis.set_drive_constants(12,0.7,0.016,5,9);// Starti = 10, I = 0.016 D = 1.3 , 1.4 , 1.6 , 1.7 , 1.9 , 2.2, 2.4, 2.6, 2.8 P = 12 , 11 , 8 , 10 , 11
  //chassis.set_heading_constants(7,0.5,0.005,6.6,10);
 /// chassis.set_heading_constants(12,0.5,0.005,6.6,10);
  //chassis.set_turn_constants(7.4,0.14,0.005,1.6,30); // D = 1.25
  chassis.set_drive_constants(12,0.5,0.05,1.8,4);
  chassis.set_turn_constants(7.4,0.18,0.005,1.6,30); 
  //chassis.set_turn_constants(12,0.19,0.013,1.2,4);
  chassis.set_heading_constants(12,0.24,0.001,4.3,4);
  task Disk2(IntakeUntilStop);

  /*Clamp.on();
  chassis.set_coordinates(-59,7,20);
  chassis.turn_to_angle(10);

  chassis.drive_distance(45);
  chassis.turn_to_angle(40);
  task testintae(IntakeUntilDisk);
  chassis.drive_distance(70);
  chassis.turn_to_angle(270);
  chassis.drive_distance(-10);
  Clamp.off();*/




  /*chassis.drive_distance(-100,0);
  chassis.turn_to_angle(270);
  chassis.set_heading(0);
  chassis.drive_distance(-100,0);
  chassis.turn_to_angle(270);
  chassis.set_heading(0);
  chassis.drive_distance(-100,0);
  chassis.turn_to_angle(270);
  chassis.set_heading(0);
  chassis.drive_distance(-100,0);
  chassis.turn_to_angle(270);
  chassis.set_heading(0);*/
  //chassis.drive_to_pose(-24,-24,90,0.5,0.5,0,12,8,2,200,3000,0.5,0.013,1.2,4,0.2,0,1.2,0); // 0.3
  /*chassis.set_coordinates(-59,7,15);
  task testintae(IntakeUntilDisk);
  chassis.drive_to_pose(-45,78,0,0.4,0.2,0,12,12,2,200,3000,0.5,0.013,1.5,4,0.2,0,4,0);
  wait(0.3,seconds);
  chassis.drive_distance(-24);
  chassis.turn_to_angle(90);
  Lift.off();
  chassis.drive_timeout = 800;
  chassis.drive_distance(-22);
  chassis.drive_timeout = 3000;
  Intake.spin(forward);
  wait(0.5,seconds);
  Intake.stop();
  Lift.on();
  chassis.drive_to_pose(-30,78,45,0.4,0.2,0,12,12,2,200,3000,0.5,0.013,1.5,4,0.2,0,4,0);*/

 // chassis.drive_to_point(-50,65,0,10,12,2,200,3000,0.5,0.013,1.2,4,0.2,0,0,0);
  //ici //chassis.drive_to_pose(-24,-24,270,0.4,0.2,0,12,12,2,200,3000,0.5,0.013,1.2,4,0.2,0,4,0); // 0.3 // d = 0, 0.3
  //ici //chassis.drive_to_pose(0,0,0,0.4,0.2,0,12,12,2,200,3000,0.5,0.013,1.2,4,0.2,0,4,0); // 0.3 // d = 0, 0.3
  //chassis.drive_to_pose(0,0,0,0.01,0.1,0,12,12,2,200,3000,0.5,0.013,1.2,4,0.2,0,4,0); // 0.3 // d = 0, 0.3
  //chassis.drive_distance(-4);
  //chassis.drive_distance(4);
  /*chassis.turn_to_angle(4z);

  chassis.drive_to_point(-24,-24,0,12,12,2,200,5000,0.7,0,7,3,0.5,0.0,6,10);
  chassis.turn_to_angle(0);
  chassis.drive_to_point(0,0,0,12,12,2,200,5000,0.7,0,7,3,0.5,0,6,10);
  chassis.turn_to_angle(0);*/

  /*chassis.drive_to_point(-39,94,12,12,12);
  chassis.drive_to_point(0,107,12,12,12);
  chassis.drive_to_point(39,91,12,12,12);
  chassis.drive_to_point(39,48,12,12,12);
  chassis.drive_to_point(0,0,12,12,12);*/

  /*chassis.set_heading_constants(7,0.5,0.005,6.6,10); //5 // 2 // 1.5 D: 6.6 I: 0.005
  chassis.set_heading(160);
  chassis.drive_distance(14,160);
  chassis.turn_to_angle(120);
  chassis.drive_distance(300,80);
 // chassis.drive_to_point(0,12);




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
  MoteurBras.setVelocity(100,percent);
  Intake_moteur.setVelocity(100,percent);
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
  chassis.drive_to_pose(6,9.6,270,0.01,0,0,12,12);
  chassis.drive_distance(-9,270);
  Clamp.off();
  wait(0.3,seconds);

  // va chercher le premier disque et le deuxieme disque
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 0;
  chassis.turn_to_angle(0);
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 150;
  chassis.drive_to_pose(21,30,0,0.5,1,9,12,12); /// 34

  chassis.turn_to_angle(90);
  chassis.drive_settle_time = 50;
  chassis.drive_distance(27,90);
  chassis.drive_settle_time = 250;

  // va chercher les 3 disques dans le coin 

  chassis.drive_to_pose(61.5,18,180,0.5,4,9,12,12);
  chassis.turn_to_angle(270);
  chassis.drive_distance(12,282);
  wait(0.3,seconds);
  chassis.drive_distance(-7,270);
  chassis.turn_to_angle(205);
  chassis.drive_distance(12,205);

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
  chassis.drive_to_point(46,30,12,12,12);
  chassis.drive_to_pose(56.8,56.8,0,0.5,1,0,12,12); // 56
  //chassis.drive_to_pose(57,57,0,0.5,1,0,12,12);
  printf("pos X: %f pos y: %f",chassis.get_X_position(),chassis.get_Y_position());
  chassis.drive_settle_error = 2.6;
  wait(0.3,seconds);
  Clamp.off();
  chassis.turn_to_angle(270);
  Intake.stop();
  Lift.off();
  chassis.drive_distance(-11,270,12,12,2.6,250,500);
  chassis.drive_timeout = 3000;
  Intake.stop();
  wait(0.3,seconds);
  MoteurBras.spin(forward);
  wait(0.7,seconds); // 1
  MoteurBras.stop();
  Lift.on();

  // va vers le point 0,0 pour la deuxieme séquence laurtre bord

  chassis.drive_distance(7,0);
  Clamp.on(); 
  chassis.turn_to_angle(90);
  chassis.drive_settle_error = 0.5;
  chassis.drive_settle_time = 50;
  chassis.drive_to_pose(-3,13.4,90,-0.5,16,0,12,12);
  chassis.drive_settle_error = 3;
  chassis.drive_settle_time = 50;
  chassis.drive_distance(-19,90,12,12);

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
  chassis.drive_distance(24,90);

  // va chercher les 3 disques dans le coin 

  chassis.drive_settle_error = 2;

  chassis.drive_to_pose(-59.5,18,180,0.5,4,9,12,12);
  chassis.drive_settle_time = 50;
  chassis.drive_timeout = 2500;
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
  chassis.drive_settle_time = 3000;
  chassis.drive_settle_time = 250;
  chassis.drive_settle_error = 0.1;
  task Disk2(IntakeUntilDisk);
  chassis.drive_to_pose(-56.5,54.5,0,0.5,4,0,9,9); // 54
  chassis.drive_settle_error = 2.6;
  wait(0.3,seconds);
  Clamp.off();
  chassis.turn_to_angle(270);
  wait(0.3,seconds);
  Intake.stop();
  Lift.off();
  chassis.drive_timeout = 500;
  chassis.drive_distance(-12,270);
  wait(0.5,seconds);
  MoteurBras.spin(forward);
  wait(0.5,seconds);
  MoteurBras.stop();
  Lift.on();
  chassis.drive_timeout = 3000;


  // va chercher le beaux disque a raph et va prendre lautre but weird


  // début de l'autre bord
  Clamp.on();
  chassis.AngleReversed = false;
  task disk3(IntakeUntilDisk);
  wait(0.3,seconds);
  chassis.drive_to_pose(-19,80,45,0.5,1,0,12,12);
  chassis.turn_to_angle(220);
  chassis.drive_distance(-20,220);
  Clamp.off();
  Intake.spin(forward);
  wait(0.3,seconds);

  // va chercher les 3 disques rouge du coins avec les disques bleu sur le top

  chassis.drive_to_pose(-55,82,270,0.5,1,0,12,12);
  chassis.drive_settle_time = 50;
  chassis.drive_distance(-13,0);
  chassis.turn_to_angle(0);
  chassis.drive_distance(24,0);
  Intake.stop();
  chassis.drive_distance(15,75);
  Intake.spin(forward);
  chassis.drive_distance(-20,0);
  chassis.drive_timeout = 1600;
  chassis.drive_to_pose(chassis.get_X_position() + 6,chassis.get_Y_position() + 20,0,0.5,1,0,12,12);
  chassis.drive_distance(-6,0);
  chassis.turn_to_angle(270);
  chassis.drive_distance(12,270);
  chassis.turn_to_angle(160);
  Clamp.on();
  chassis.drive_distance(-14,160,12,12,2.5,300,1000);
  chassis.drive_timeout = 3000;
  Intake.stop();
  Intake.spinFor(reverse,0.7,seconds);
  chassis.drive_distance(14,160);

  // va chercher le disque random et le mets dans le alliance stake bleu
  Intake.spin(forward);
  /*chassis.drive_to_pose(28,68,180,0.5,1,0,12,12);
  Intake.stop();
  task disk4(IntakeUntilDisk);
  Clamp.off();
  chassis.drive_timeout = 2000;
  chassis.drive_to_pose(8,115,0,0.5,1,0,12,12);
  Intake.spin(forward);
  wait(0.3,seconds);
  chassis.turn_to_angle(90);
  */
  chassis.drive_distance(300,80);  
}


void Skill2(){
  //chassis.set_heading_constants(7,0.5,0.05,5.5,60);
  MoteurBras.setVelocity(120,percent);
  Intake_moteur.setVelocity(120,percent);
  chassis.set_drive_constants(12,0.7,0.016,1.3,10);
 // chassis.set_turn_constants(7,0.14,0.005,1.4,30); // D = 1.25
  chassis.set_heading_constants(7,0.5,0.005,6.6,10);
  chassis.set_drive_constants(12,0.5,0.05,1.8,4);
  chassis.drive_settle_time = 150;
  chassis.turn_settle_time = 150;
  Clamp.off();
  Intake.spin(forward);
  wait(0.5,seconds);
  Clamp.on();
  chassis.drive_distance(4,0);

  // va chercher le premier but

  chassis.turn_to_angle(180);

  chassis.drive_distance(-13,270);
  chassis.drive_distance(-8.4,270);
  Clamp.off();
  wait(0.3,seconds);
  

  // va chercher le premier disque et le deuxieme disque
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 0;
  chassis.turn_to_angle(0);
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 150;
  chassis.drive_distance(26,30);

  chassis.turn_to_angle(90);
  chassis.drive_settle_time = 50;
  chassis.drive_distance(29,90);
  chassis.drive_settle_time = 250;

  // va chercher les 3 disques dans le coin 
  chassis.drive_min_voltage = 0;
  chassis.drive_settle_time = 50;
  chassis.drive_settle_error = 2.6;
  chassis.turn_settle_time = 50;
  chassis.turn_settle_error = 3;
  chassis.turn_to_angle(175);
  chassis.drive_distance(20,175);
  chassis.turn_to_angle(270);
  chassis.drive_distance(12,282);
  wait(0.3,seconds);
  chassis.drive_distance(-7,270);
  chassis.turn_to_angle(210);
  chassis.drive_distance(9,210); // 8.4
 
  // va dans le coin et dépose le but
  wait(0.3,seconds);
  chassis.drive_distance(-5,325);
  Clamp.on();
  chassis.drive_timeout = 2500;
  chassis.drive_distance(-10,325);
  Intake.stop();

  // prendre le ring et le mettre sur le top truc
  chassis.drive_settle_time = 400;
  chassis.drive_settle_error = 1;
  wait(0.2,seconds);
  task Disk(IntakeUntilDisk);
  chassis.drive_timeout = 2500;
  chassis.drive_distance(58,5); // 62 //20
  chassis.turn_to_angle(270);
  Clamp.off();
  Lift.off();
  chassis.drive_timeout = 600;
  chassis.drive_distance(-15,270);
  wait(0.4,seconds);
  MoteurBras.spin(forward);
  wait(0.7,seconds); // 1
  MoteurBras.stop();
  Lift.on();
  Clamp.on();

  // va vers le point 0,0 pour la deuxieme séquence laurtre bord
  chassis.drive_timeout = 3000;
  chassis.drive_min_voltage = 0;
  chassis.drive_settle_time = 50;
  chassis.drive_settle_error = 1.4;
  chassis.turn_settle_time = 50;
  chassis.turn_settle_error = 3;
  chassis.drive_to_point(0,11.5,0,12,12,2.3,200,3000,0.7,0,7,3,0.5,0.0,4,10);
  chassis.DriveL.stop();
  chassis.DriveR.stop();
  chassis.turn_to_angle(90);
  chassis.drive_timeout = 2000;
  chassis.drive_distance(-20,90);
  Clamp.off();
  wait(0.4,seconds);

  // se tourne pour faire les même mouvement mais a l'inverse

  chassis.turn_to_angle(0);


  // legit la meme chose que au début mais a l'inverse

  //////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////

  // va chercher le premier disque et le deuxieme disque
  Intake.spin(forward);
  chassis.AngleReversed = true;
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 0;
 // chassis.turn_to_angle(0);
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 150;
  chassis.drive_distance(22,0);
  chassis.turn_to_angle(90);
  chassis.drive_distance(36,90); // 34.8
  chassis.turn_to_angle(180);
  chassis.drive_distance(21,180);
  chassis.turn_to_angle(270);
  chassis.drive_distance(13,285);
  wait(0.3,seconds);
  chassis.drive_distance(-7,270);
  chassis.turn_to_angle(198); // 195
  chassis.drive_distance(11,198); // 9

  // va dans le coin et dépose le but

  chassis.drive_distance(-5,310);
  Clamp.on();
  chassis.drive_distance(-6,310);
  Intake.stop();
// code a mathis = 2710
  // aller sur le top truc
  task Disk2(IntakeUntilDisk);
  chassis.drive_distance(58.2,18); //56.6,17
  chassis.turn_to_angle(270);
  Clamp.off();
  Lift.off();
  chassis.drive_timeout = 400;
  chassis.drive_distance(-16,270);
  wait(0.4,seconds);
  MoteurBras.spin(forward);
  wait(0.7,seconds); // 1
  MoteurBras.stop();
  Lift.on();
  Clamp.on();
  chassis.drive_timeout = 2500;
  chassis.drive_distance(2,270);

  // va lautre bord vers le ring weird
  double waittime = 1;
  task take3(IntakeUntilDiskButWait, (void *)&waittime);
 // task Disk3(IntakeUntilDisk);
  chassis.AngleReversed = false;
  chassis.drive_to_point(-19,80,0,12,12,2.3,200,3000,0.7,0,7,3,0.5,0.0,4,10); // 76
  chassis.drive_distance(12,37); //14
  chassis.turn_to_angle(240);
  chassis.drive_distance(-20,215);
  Clamp.off();
  Intake.spin(forward);
  wait(0.5,seconds);

  // va chercher le premier disque du coin bleu
  chassis.drive_to_pose(-55,82,270,0.5,1,0,12,12);
  chassis.drive_settle_time = 50;
  chassis.drive_distance(-10,0);
  chassis.turn_to_angle(0);
  chassis.drive_distance(26,0);
  Intake.stop();
  chassis.drive_distance(15,75);
  Intake.spin(forward);
  chassis.drive_distance(-20,0);
  chassis.drive_timeout = 1600;
  chassis.drive_to_pose(chassis.get_X_position() + 6,chassis.get_Y_position() + 20,0,0.5,1,0,12,12);
  chassis.drive_distance(-9,0);
  chassis.turn_to_angle(270);
  chassis.drive_distance(12,270);
  chassis.turn_to_angle(160);
  Clamp.on();
  chassis.drive_distance(-20,160,12,12,2.5,300,1000);
  chassis.drive_timeout = 3000;
  Intake.stop();
  Intake.spinFor(reverse,0.2,seconds);

 // chassis.drive_to_point(-56,82);
  
  /*chassis.turn_to_angle(0);
  chassis.drive_distance(20);
  chassis.drive_distance(26,90);
  chassis.drive_distance(-35,0);
  chassis.drive_distance(18,90);*/

  // va le mettre dans le coin 
  wait(0.3,seconds);
  chassis.turn_to_angle(120);
  Clamp.on();
  Intake.spin(reverse);
  // va chercher prendre le ring et le mets sur le alliance stake
  chassis.drive_timeout = 3000;
  chassis.drive_distance(110,80); 
  chassis.AngleReversed = false;
  Climb.on();
  chassis.drive_to_point(30,70,70,12,12,2.3,200,3000,0.7,0,7,3,0.5,0.0,4,10); // 76
  chassis.drive_to_point(0,63,70,12,12,2.3,200,3000,0.7,0,7,3,0.5,0.0,4,10); // 76
}

// AWP Solo (7 pts) Rouge Côté Gauche

void Rouge_Gauche_WP(){
  Clamp.off();
  chassis.drive_settle_time = 150;
  chassis.turn_settle_time = 150;
  chassis.drive_distance(-9.9, 0, 12, 5);
  chassis.turn_to_angle(90, 12);
  chassis.drive_timeout = 600;
  chassis.drive_distance(-9, 90, 12, 5);
  chassis.drive_timeout = 2000;
  Intake.setVelocity(100, percent);
  Intake.spinFor(forward, 1, seconds);
  Intake.setVelocity(100, percent);

  chassis.drive_distance(18, 60, 12, 8);
  Clamp.on();
  chassis.turn_to_angle(230, 12);
  chassis.drive_distance(-17, 230, 12, 8);
  Clamp.off();
  chassis.turn_to_angle(0,12);
  chassis.drive_to_pose(36,15.5,0,0.5,1,6);
  Intake.spin(forward);
  chassis.drive_distance(10,0,12,5);
  chassis.turn_to_angle(90);
  chassis.drive_distance(10, 90, 12, 8);
  chassis.drive_to_pose(44,4,90,0.5,1,0);
  wait(3,seconds);
  Lift.on();
  Clamp.on();
  Intake.stop();
  wait(20,seconds);
  
}

void Skill3(){
  //chassis.set_heading_constants(7,0.5,0.05,5.5,60);
  SetTeam(1);
  MoteurBras.setVelocity(120,percent);
  Intake_moteur.setVelocity(120,percent);
  //chassis.set_drive_constants(11,0.7,0.016,3,9);// Starti = 10, I = 0.016 D = 1.3 , 1.4 , 1.6 , 1.7 , 1.9 , 2.2, 2.4, 2.6, 2.8 P = 12 , 11 , 8 , 10 , 11
  //chassis.set_heading_constants(7,0.5,0.005,6.6,10);
  //chassis.set_turn_constants(7,0.14,0.005,1.4,30); // D = 1.25
  chassis.set_drive_constants(12,0.7,0.016,3,9);// Starti = 10, I = 0.016 D = 1.3 , 1.4 , 1.6 , 1.7 , 1.9 , 2.2, 2.4, 2.6, 2.8 P = 12 , 11 , 8 , 10 , 11
  //chassis.set_drive_constants(12,0.7,0.016,5,9);// Starti = 10, I = 0.016 D = 1.3 , 1.4 , 1.6 , 1.7 , 1.9 , 2.2, 2.4, 2.6, 2.8 P = 12 , 11 , 8 , 10 , 11
  chassis.set_heading_constants(12,0.5,0.005,6.6,10);
  chassis.set_turn_constants(12,0.14,0.005,1.6,30); 
  chassis.drive_settle_time = 150;
  chassis.turn_settle_time = 150;
  Clamp.off();
  Intake.spin(forward);
  wait(0.5,seconds);
  Clamp.on();
  chassis.drive_distance(4,0);

  // va chercher le premier but

  chassis.turn_to_angle(180);

  chassis.drive_distance(-8,270); // -5.3
  chassis.drive_distance(-6,270); // 9.9
  Clamp.off();
  wait(0.3,seconds);
  

  // va chercher le premier disque et le deuxieme disque
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 0;
  chassis.turn_to_angle(0);
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 150;
  chassis.drive_distance(26,30);

  chassis.turn_to_angle(90);
  chassis.drive_settle_time = 50;
  chassis.drive_distance(30,90);
  chassis.drive_settle_time = 250;

  // va chercher les 3 disques dans le coin 
  chassis.drive_min_voltage = 0;
  chassis.drive_settle_time = 50;
  chassis.drive_settle_error = 2.6;
  chassis.turn_settle_time = 50;
  chassis.turn_settle_error = 3;
  chassis.turn_to_angle(170);
  chassis.drive_distance(20,170);
  chassis.turn_to_angle(270);
  chassis.drive_distance(17,282);
  //wait(0.3,seconds);sssss
  chassis.drive_distance(-12,270);
  chassis.turn_to_angle(225); // 220
  chassis.drive_distance(12.4,225); //8.4 10.4
 
  // va dans le coin et dépose le but
  wait(0.3,seconds);
  chassis.drive_distance(-5,310);
  Clamp.on();
  wait(0.2,seconds);
  chassis.drive_distance(-8,310);
  Intake.stop();

  // prendre le ring et le mettre sur le top truc
  chassis.drive_settle_time = 400;
  chassis.drive_settle_error = 1;
  wait(0.2,seconds);
  chassis.drive_timeout = 2500;
  task Disk(IntakeUntilDisk);
  chassis.drive_distance(60.7,14);
  chassis.turn_to_angle(270);
  Clamp.off();
  Lift.off();
  chassis.drive_timeout = 600;
  chassis.drive_distance(-15,270);
  wait(0.4,seconds);
  MoteurBras.spin(forward);
  wait(0.7,seconds); // 1
  MoteurBras.stop();
  Lift.on();
  Clamp.on();

  // va vers le point 0,0 pour la deuxieme séquence laurtre bord
  chassis.drive_timeout = 3000;
  chassis.drive_min_voltage = 0;
  chassis.drive_settle_time = 50;
  chassis.drive_settle_error = 1.4;
  chassis.turn_settle_time = 50;
  chassis.turn_settle_error = 3;
  chassis.drive_to_point(0,13,0,12,12,2.3,200,3000,0.7,0,7,3,0.5,0.0,4,10);
  chassis.DriveL.stop();
  chassis.DriveR.stop();
  chassis.turn_to_angle(90);
  chassis.drive_timeout = 2000;
  chassis.drive_distance(-14,90);
  Clamp.off();
  wait(0.4,seconds);

  // se tourne pour faire les même mouvement mais a l'inverse

  chassis.turn_to_angle(0);


  // legit la meme chose que au début mais a l'inverse

  //////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////

  // va chercher le premier disque et le deuxieme disque
  Intake.spin(forward);
  chassis.AngleReversed = true;
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 0;
  chassis.turn_to_angle(390);
  chassis.turn_settle_error = 3;
  chassis.turn_settle_time = 150;
  chassis.drive_distance(22,390);
  chassis.turn_to_angle(90);
  chassis.drive_distance(34,90);
  chassis.turn_to_angle(170);
  chassis.drive_distance(21,170);
  chassis.turn_to_angle(270);
  chassis.drive_distance(13,280);
  //wait(0.3,seconds);
  chassis.drive_distance(-7,270);
  chassis.turn_to_angle(220); // 200
  chassis.drive_distance(9,220);

  // va dans le coin et dépose le but

  chassis.drive_distance(-5,310);
  Clamp.on();
  chassis.drive_distance(-6,310);
  Intake.stop();

  // aller sur le top truc
  task Disk2(IntakeUntilDisk);
  chassis.drive_distance(53,16);
  //chassis.drive_to_point(-55.5,63,0,12,12);
 // chassis.drive_distance(56.6,17);
  chassis.turn_to_angle(270);
  Clamp.off();
  Lift.off();
  chassis.drive_timeout = 200;
  chassis.drive_distance(-16,270);
  wait(0.4,seconds);
  MoteurBras.spin(forward);
  wait(0.7,seconds); // 1
  MoteurBras.stop();
  Lift.on();
  Clamp.on();
  chassis.drive_timeout = 2500;
  chassis.drive_distance(2,270);

  // va lautre bord vers le ring weird
  task Disk3(IntakeUntilDisk);
  chassis.AngleReversed = false;
  chassis.drive_to_point(-19,84,0,12,12,2.3,200,3000,0.7,0,7,3,0.5,0.0,4,10);
  chassis.drive_distance(14,chassis.get_absolute_heading());
  chassis.turn_to_angle(220);
  chassis.drive_distance(-20,220);
  Clamp.off();
  Intake.spin(forward);
  wait(0.5,seconds);

  // va chercher le premier disque du coin bleu
  chassis.drive_to_pose(-55,82,270,0.5,1,0,12,12,2.3,200,3000,0.7,0,7,3,0.5,0.0,4,10);
  chassis.drive_settle_time = 50;
  chassis.drive_distance(1,270);
  chassis.turn_to_angle(380);
  chassis.drive_distance(20,0);
  chassis.drive_distance(12,90);
  chassis.turn_to_angle(180);
  chassis.drive_distance(20,180);
  /*chassis.drive_distance(-13,0);
  chassis.turn_to_angle(0);
  chassis.drive_distance(26,0);
  Intake.stop();
  chassis.drive_distance(15,75);
  Intake.spin(forward);
  chassis.drive_distance(-20,0);
  chassis.drive_timeout = 1600;
  chassis.drive_to_pose(chassis.get_X_position() + 6,chassis.get_Y_position() + 20,0,0.5,1,0,12,12);
  chassis.drive_distance(-5,0);
  chassis.turn_to_angle(270);
  chassis.drive_distance(12,270);
  */chassis.turn_to_angle(160);
  Clamp.on();
  chassis.drive_distance(-14,160,12,12,2.5,300,1000);
  chassis.drive_timeout = 3000;
  Intake.stop();
  Intake.spinFor(reverse,0.2,seconds);

 // chassis.drive_to_point(-56,82);
  
  /*chassis.turn_to_angle(0);
  chassis.drive_distance(20);
  chassis.drive_distance(26,90);
  chassis.drive_distance(-35,0);
  chassis.drive_distance(18,90);*/

  // va le mettre dans le coin 
  wait(0.3,seconds);
  chassis.turn_to_angle(120);
  Clamp.on();
  Intake.spin(reverse);
  // va chercher prendre le ring et le mets sur le alliance stake
  chassis.drive_distance(10,110);
  chassis.drive_timeout = 3000;
  chassis.drive_distance(300,80);  
}

// 2 top ring + échelle (6 pts) Rouge Côté Droit

void Rouge_Droite(){
  chassis.drive_distance(-9.9, 0, 7, 5);
}

void Bleu_Gauche_Rush_Goal(){
  //chassis.X_Reversed = true;
  //chassis.turn_to_angle(330);
  //chassis.drive_to_pose(0,-10,150,0.5,0,0,12,10);
  SetTeam(0);
  chassis.drive_settle_time = 0;
  chassis.drive_to_pose(9,-42,150,0.5,-4,0,12,10,2.3,200,3000,0.7,0,7,3,0.5,0.0,4,10);
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
  chassis.drive_to_pose(26,6,90,0.5,9,0,12,10);
  chassis.turn_to_angle(50);
  chassis.drive_distance(24,50);
  wait(0.6,seconds);
  chassis.turn_to_angle(180);
  //chassis.drive_distance(-40,chassis.get_absolute_heading());
  //chassis.drive_to_pose(-20,-38,180,0.5,-6,0,12,10);
  Intake.stop();
  chassis.drive_distance(12,180);
  Clamp.on();
  chassis.drive_distance(18,180);
  chassis.turn_to_angle(0);

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
  chassis.drive_to_pose(-26,6,90,0.5,9,0,12,10);
  chassis.turn_to_angle(50);
  chassis.drive_distance(24,50);
  wait(0.6,seconds);
  chassis.turn_to_angle(180);
  chassis.drive_distance(12,180);
  Clamp.on();
  chassis.drive_distance(18,180);
  chassis.turn_to_angle(0);
 // chassis.drive_to_pose(20,-38,180,0.5,-6,0,12,10);



  //Intake.stop();
}

void RougeGaucheFinal(){

  SetTeam(1);
  // va chercher le premier but
  chassis.drive_settle_time = 0;
  chassis.drive_distance(-23); // - 26
  Clamp.off();
  wait(0.35,seconds);
  chassis.turn_settle_time = 50;
  chassis.turn_to_angle(120);
  chassis.drive_settle_time = 50;
  // va chercher le premier disque
  
  Intake.spin(forward);
  //task IntakeUntilStop3(IntakeUntilStop);
  chassis.drive_distance(21,115);
  wait(0.5,seconds);
  chassis.drive_distance(-18,120);

  // va chercher le deuxième ring

  chassis.turn_to_angle(65);
  chassis.drive_distance(16,65);

  // va chercher le 3ième disque

  chassis.turn_to_angle(130);
  chassis.drive_distance(17,130);
  wait(0.3,seconds);
  chassis.turn_to_angle(0);

  // aller dans le coin blablbalblalblalba

  chassis.drive_to_pose(20,12,20,0.6,1,9,12,12);
  chassis.drive_distance(5,chassis.get_absolute_heading(),12,12,2.6,50,2000);
  chassis.drive_settle_error = 0.5;
  chassis.drive_distance(-1,chassis.get_absolute_heading(),12,12,2.6,50,2000);
  chassis.drive_distance(7,chassis.get_absolute_heading(),12,12,2.6,50,2000);
  wait(0.9,seconds);
  chassis.drive_to_pose(-90,-8,300,0.5,1,12,12,12);
};

void RougeDroiteFinal(){

  // va prendre le but

  chassis.drive_settle_time = 120;
  chassis.drive_distance(-24);
  Clamp.off();
  wait(0.2,seconds);
  Intake.spin(forward);

  // va chercher le premier disque
  
  chassis.turn_to_angle(290);
  chassis.drive_distance(20);
  chassis.turn_to_angle(50);

  // va chercher les disque dans le coin
 /* chassis.drive_to_pose(-12,4.5,50,0.5,1,8,12,12);
  chassis.turn_to_angle(325);
  chassis.drive_distance(17,325,12,12,2.6,50,2000);
  chassis.drive_distance(-5,325,12,12,2.6,50,2000);
  chassis.drive_distance(6,325,12,12,2.6,50,2000);*/

  chassis.drive_to_pose(13,-24,50,0.5,1,8,12,12);

}


void BleuDroiteFinal(){
  SetTeam(0);
  chassis.AngleReversed = true;
  // va chercher le premier but
  chassis.drive_settle_time = 0;
  chassis.drive_distance(-23); // - 26
  Clamp.off();
  wait(0.35,seconds);
  chassis.turn_settle_time = 50;
  chassis.turn_to_angle(120);
  chassis.drive_settle_time = 50;
  // va chercher le premier disque
  
  Intake.spin(forward);
  //task IntakeUntilStop3(IntakeUntilStop);
  chassis.drive_distance(21,115);
  wait(0.5,seconds);
  chassis.drive_distance(-18,120);

  // va chercher le deuxième ring

  chassis.turn_to_angle(65);
  chassis.drive_distance(16,65);

  // va chercher le 3ième disque

  chassis.turn_to_angle(130);
  chassis.drive_distance(17,130);
  wait(0.3,seconds);
  chassis.turn_to_angle(0);

  // aller dans le coin blablbalblalblalba

  chassis.drive_to_pose(-20,12,20,0.6,1,9,12,12);
  chassis.drive_distance(5,chassis.get_absolute_heading(),12,12,2.6,50,2000);
  chassis.drive_settle_error = 0.5;
  chassis.drive_distance(-1,chassis.get_absolute_heading(),12,12,2.6,50,2000);
  chassis.drive_distance(7,chassis.get_absolute_heading(),12,12,2.6,50,2000);
  wait(0.9,seconds);
  chassis.drive_to_pose(90,-8,300,0.5,1,12,12,12);
};

void BleuGaucheFinal(){

  // va prendre le but
  chassis.AngleReversed = true;
  chassis.drive_settle_time = 120;
  chassis.drive_distance(-24);
  Clamp.off();
  wait(0.2,seconds);
  Intake.spin(forward);

  // va chercher le premier disque
  
  chassis.turn_to_angle(290);
  chassis.drive_distance(20,290);
  chassis.turn_to_angle(50);

  // va chercher les disque dans le coin
  /*chassis.drive_to_pose(12,4.5,50,0.5,1,8,12,12);
  chassis.turn_to_angle(325);
  chassis.drive_distance(17,325,12,12,2.6,50,2000);
  chassis.drive_distance(-5,325,12,12,2.6,50,2000);
  chassis.drive_distance(6,325,12,12,2.6,50,2000);*/

  chassis.drive_to_pose(-13,-24,50,0.5,1,8,12,12);
}

void SignatureDroiteWinRouge(){
  //chassis.set_heading_constants(7,0.5,0.005,6.6,10);
  chassis.drive_max_voltage = 12;
  chassis.AngleReversed = true;
  //chassis.drive_distance(-50,280);
  chassis.drive_to_pose(-9.5,-43,150,0.5,-4,0,12,10);
  chassis.drive_settle_time = 100;
  Clamp.off();
  chassis.drive_settle_time = 100;
  double waittime = 0.3;
  task take(IntakeButWait, (void *)&waittime);
  chassis.turn_settle_time = 100;
  chassis.drive_distance(5,chassis.get_absolute_heading());
  Intake.stop();
  Intake.spin(forward);
  //chassis.turn_to_angle(30,9);
  //chassis.drive_distance(12,30);
  task disk8(IntakeUntilDisk);
  chassis.turn_to_angle(280);
  //wait(0.3,seconds);
  Clamp.on();
  chassis.drive_distance(3,280);
  chassis.turn_to_angle(130);
  chassis.drive_distance(-20,130);
  //chassis.drive_to_pose(10,-24,270,0.5,-6,0,12,10);
  Clamp.off();
  Intake.spin(forward);

  ////
  chassis.turn_to_angle(350);
  chassis.drive_to_point(59,-14,12,12,12);
  Intake.stop();
  task disk9(IntakeUntilDisk);
 // chassis.drive_to_point(55,-14,12,12,12);
  chassis.drive_distance(33,240);
  wait(0.2,seconds);
  Clamp.on();
  chassis.turn_to_angle(210);

  ///

  chassis.turn_to_angle(270);
  chassis.drive_distance(-30,270);
  Clamp.off();
  Intake.spin(forward);

  chassis.drive_distance(20,190);
  chassis.drive_distance(-38,330);


}

void SignatureGaucheWinRouge(){
  SetTeam(1);

  // premier disque


  task disk7(IntakeUntilDisk);
  chassis.drive_distance(45,0);


  // le but

  chassis.drive_distance(-24,290);
  Clamp.off();
  wait(0.2,seconds);

  // le deuxième disque

  Intake.spin(forward);

  chassis.drive_distance(30,290);

  // va cherher le disque

  chassis.turn_to_angle(120);

  task disk8(IntakeUntilDisk);
  chassis.drive_to_point(0,2,12,12,12);
  Clamp.on();
  chassis.drive_to_point(26.5,-10);
  chassis.turn_to_angle(25);
  Clamp.off();
  // met le truc sur le alliance stake

  chassis.drive_timeout = 600;
  chassis.drive_distance(-20,20);
  chassis.drive_timeout = 2000;
  Intake.spin(forward);
  wait(0.5,seconds);
  // va chercher un autre but
  chassis.drive_distance(3,90);
  chassis.turn_to_angle(90);
  chassis.drive_to_point(50,10,0,12,12);

  chassis.turn_to_angle(250);
  chassis.drive_distance(-20,250);
  Clamp.off();





}

void UNUSEDProvincialRougeDroiteWP(){
  SetTeam(1);
  chassis.set_drive_constants(12,0.7,0.016,5,9);// Starti = 10, I = 0.016 D = 1.3 , 1.4 , 1.6 , 1.7 , 1.9 , 2.2, 2.4, 2.6, 2.8 P = 12 , 11 , 8 , 10 , 11
  chassis.set_heading_constants(12,0.5,0.005,6.6,10);
  chassis.set_turn_constants(7.4,0.14,0.005,1.6,30); 

  /*chassis.set_drive_constants(11,0.7,0.016,3,9);// Starti = 10, I = 0.016 D = 1.3 , 1.4 , 1.6 , 1.7 , 1.9 , 2.2, 2.4, 2.6, 2.8 P = 12 , 11 , 8 , 10 , 11
  chassis.set_heading_constants(7,0.5,0.005,6.6,10);
  chassis.set_turn_constants(7,0.14,0.005,1.4,30); // D = 1.25*/

  Clamp.off();
  chassis.set_heading(90);
  chassis.drive_distance(-15,90);

  // aliance stake
  chassis.turn_to_angle(0);
  chassis.drive_timeout = 500;
  chassis.drive_distance(-14);
  chassis.drive_timeout = 3000;
  Intake.spin(forward);
  wait(0.3,seconds);
  
  // va prendre le but
  Clamp.on();
  chassis.drive_to_point(-5,9,0,12,12,2,200,5000,0.7,0,7,3,0.5,0.0,6,10);
  chassis.turn_to_angle(221);
  chassis.drive_distance(-12.4);
  Clamp.off();
  wait(0.3,seconds);

  // va pogner le premier ring 

  chassis.turn_to_angle(80);
  Intake.spin(forward);
 // task IntakeStop(IntakeUntilStop);
  chassis.drive_distance(33);

  // drop le but

  chassis.drive_distance(-17);
  Clamp.on();
  chassis.turn_to_angle(200);
  StopIntake();

  // va prendre lautre but

  chassis.drive_distance(-28);
  Clamp.off();
  Intake.stop();
}

void ProvRougeGaucheSoloWP() {
  SetTeam(0);
  chassis.set_drive_constants(12,0.7,0.016,3,9);// Starti = 10, I = 0.016 D = 1.3 , 1.4 , 1.6 , 1.7 , 1.9 , 2.2, 2.4, 2.6, 2.8 P = 12 , 11 , 8 , 10 , 11
  //chassis.set_drive_constants(12,0.7,0.016,5,9);// Starti = 10, I = 0.016 D = 1.3 , 1.4 , 1.6 , 1.7 , 1.9 , 2.2, 2.4, 2.6, 2.8 P = 12 , 11 , 8 , 10 , 11
  chassis.set_heading_constants(12,0.5,0.005,6.6,10);
  chassis.set_turn_constants(7.4,0.14,0.005,1.6,30); 
  chassis.turn_settle_time = 0;

  chassis.set_heading(270);
  Clamp.off();
  chassis.turn_settle_time = 100;
  chassis.drive_distance(-11);
  chassis.turn_to_angle(0);
  chassis.drive_timeout = 500;
  chassis.drive_distance(-16); //-16
  chassis.drive_timeout = 2500;
  chassis.drive_settle_time = 50;
  Intake.spin(forward);
  wait(0.3,seconds);
  Intake.stop();
  Clamp.on();
  Intake.spin(forward);
  chassis.drive_distance(8.5,330);
  chassis.turn_to_angle(147);
  //chassis.drive_to_pose(0.34,11,0,0.5,1,0,12,12,2,100,5000,0.7,0,7,3,0.5,0.0,4,10); // 6

  //chassis.drive_to_point(-4,11,0,12,12,2,200,5000,0.7,0,7,3,0.5,0.0,6,10);
  Intake.spin(reverse);
  chassis.drive_settle_time = 0;
  chassis.drive_distance(-20,147,8,12);
  chassis.drive_max_voltage = 12;
  Clamp.off();
  Intake.stop();
  chassis.drive_settle_time = 50;
  wait(0.2,seconds);
  
  chassis.turn_to_angle(325);
  Intake.spin(forward);
  //task IntakeUntilStop2(IntakeUntilStop);
  chassis.drive_to_pose(-20,40,310,0.5,1,12,12,12,2,100,5000,0.7,0,7,3,0.5,0.0,4,10); // 6
  chassis.drive_timeout = 1600;
  chassis.drive_to_pose(-45.5,43,270,0.5,1,0,6,12,2,100,1600,0.7,0,7,3,0.5,0.0,4,10); // 6
  chassis.drive_timeout = 2500;
  chassis.drive_distance(-26,300);
  chassis.turn_to_angle(270);
  chassis.drive_distance(19);
  chassis.turn_settle_time = 0;
  chassis.turn_to_angle(270);
  chassis.turn_settle_time = 200;
  //StopIntake();
 // task IntakeUntillDisk13(IntakeUntilDisk);
  //Intake.stop();
  chassis.drive_to_pose(50,10,0,0.5,1,12,12,12,2,100,5000,1,0,7,3,0.5,0.0,4,10); // 6
  //chassis.drive_to_pose(12,16,90,0.5,1,12,12,12,2,100,5000,1,0,7,3,0.5,0.0,4,10); // 6
  //chassis.drive_to_pose(16,40,0,0.5,1,0,8,12,2,100,5000,1,0,7,3,0.5,0.0,4,10); // 6
  //Intake.spin(reverse);
}

void ProvBleuDroitSoloWP()
{
  SetTeam(1);
  chassis.set_drive_constants(12,0.7,0.016,3,9);// Starti = 10, I = 0.016 D = 1.3 , 1.4 , 1.6 , 1.7 , 1.9 , 2.2, 2.4, 2.6, 2.8 P = 12 , 11 , 8 , 10 , 11
  //chassis.set_drive_constants(12,0.7,0.016,5,9);// Starti = 10, I = 0.016 D = 1.3 , 1.4 , 1.6 , 1.7 , 1.9 , 2.2, 2.4, 2.6, 2.8 P = 12 , 11 , 8 , 10 , 11
  chassis.set_heading_constants(12,0.5,0.005,6.6,10);
  chassis.set_turn_constants(7.4,0.14,0.005,1.6,30); 
  chassis.turn_settle_time = 0;
  chassis.AngleReversed = true;
  chassis.set_heading(90);
  Clamp.off();
  chassis.turn_settle_time = 100;
  chassis.drive_distance(-11,270);
  chassis.turn_to_angle(0);
  chassis.drive_timeout = 500;
  chassis.drive_distance(-16);
  chassis.drive_timeout = 2500;
  chassis.drive_settle_time = 50;
  Intake.spin(forward);
  wait(0.3,seconds);
  Intake.stop();
  Clamp.on();
  Intake.spin(forward);
  chassis.drive_distance(8.5,330);
  chassis.turn_to_angle(147);
  //chassis.drive_to_pose(0.34,11,0,0.5,1,0,12,12,2,100,5000,0.7,0,7,3,0.5,0.0,4,10); // 6

  //chassis.drive_to_point(-4,11,0,12,12,2,200,5000,0.7,0,7,3,0.5,0.0,6,10);
  Intake.spin(reverse);
  chassis.drive_settle_time = 0;
  chassis.drive_distance(-20,147,8,12);
  chassis.drive_max_voltage = 12;
  Clamp.off();
  Intake.stop();
  chassis.drive_settle_time = 50;
  wait(0.2,seconds);

  chassis.turn_to_angle(325);
  Intake.spin(forward);
  //task IntakeUntilStop2(IntakeUntilStop);
  chassis.drive_to_pose(20,40,310,0.5,1,12,12,12,2,100,5000,0.7,0,7,3,0.5,0.0,4,10); // 6
  chassis.drive_timeout = 1600;
  chassis.drive_to_pose(45.5,43,270,0.5,1,0,6,12,2,100,1600,0.7,0,7,3,0.5,0.0,4,10); // 6
  chassis.drive_timeout = 2500;
  chassis.drive_distance(-26,300);
  chassis.turn_to_angle(270);
  chassis.drive_distance(19,270);
  chassis.turn_settle_time = 0;
  chassis.turn_to_angle(270);
  chassis.turn_settle_time = 200;
  //StopIntake();
 // task IntakeUntillDisk13(IntakeUntilDisk);
 // Intake.stop();
  chassis.drive_to_pose(-50,10,0,0.5,1,12,12,12,2,100,5000,1,0,7,3,0.5,0.0,4,10); // 6
  //chassis.drive_to_pose(-14,40,0,0.5,1,0,8,12,2,100,5000,1,0,7,3,0.5,0.0,4,10); // 6
}

void ProvRougeDroiteSoloWP() {
  SetTeam(1);
  chassis.set_drive_constants(12,0.7,0.016,3,9);// Starti = 10, I = 0.016 D = 1.3 , 1.4 , 1.6 , 1.7 , 1.9 , 2.2, 2.4, 2.6, 2.8 P = 12 , 11 , 8 , 10 , 11
  //chassis.set_drive_constants(12,0.7,0.016,5,9);// Starti = 10, I = 0.016 D = 1.3 , 1.4 , 1.6 , 1.7 , 1.9 , 2.2, 2.4, 2.6, 2.8 P = 12 , 11 , 8 , 10 , 11
  chassis.set_heading_constants(12,0.5,0.005,6.6,10);
  chassis.set_turn_constants(7.4,0.14,0.005,1.6,30); 
  Intake.spin(reverse);
};

void SkillProv(){

  SetTeam(1);

  chassis.set_drive_constants(12,0.5,0.05,1.8,4);
  chassis.set_turn_constants(10,0.2,0.013,1.3,4); // p = 0.19 percent = 7.4 d= 1.2
  chassis.set_heading_constants(12,0.24,0.001,4.3,4);
  //chassis.drive_to_point(-24,-24,0,12,8,2,200,3000,0.5,0.013,1.2,4,0.2,0,0,0); // 0.3

  // mettre le alliance stake et avancer et tourner vers le but
  chassis.drive_timeout = 2800;
  Intake.spin(forward);
  wait(0.3,seconds);
  chassis.drive_settle_time = 0;
  chassis.drive_distance(11.7);
  chassis.drive_settle_time = 50;
  chassis.turn_settle_time = 100;
  Clamp.on();
  chassis.turn_to_angle(270);

  // prendre le but

  chassis.drive_distance(-17);
  Clamp.off();

  // prendre le 1er ring et le 2ieme ring

  chassis.turn_to_angle(6);
  chassis.drive_distance(20);

  chassis.turn_to_angle(90);
  chassis.drive_distance(22);
  
  // prendre le 3ieme ring

  chassis.turn_to_angle(20);
  chassis.drive_distance(24,20,12,12);
 // wait(0.5,seconds);
  chassis.drive_distance(-25,20,12,12);
  //chassis.set_turn_constants(12,0.14,0.005,1.6,30); 
 // chassis.set_turn_constants(12,0.19,0.013,1.2,4);
 //  chassis.set_turn_constants(12,0.19,0.013,1.2,4);

 // chassis.set_turn_constants(12,0.19,0.013,1.6,4);
  //chassis.set_turn_constants(7.4,0.2,0.013,2,4); // p = 0.19
  chassis.set_turn_constants(10,0.18,0.005,1.6,30); // percent = 7.4
  chassis.turn_to_angle(180,10);

  // prendre le 4 ieme et 5 ring;

  chassis.drive_distance(29,180,6,12);
  //wait(0.5,seconds);

  // prendre le 6 ieme ring

  chassis.drive_distance(-14,180,12,12);
  Intake.spin(reverse);
  chassis.turn_to_angle(90);
  Intake.spin(forward);
  chassis.drive_distance(10,90,12,12);
  wait(0.3,seconds);

  // mets le but dans le coin

  chassis.turn_to_angle(345);
  Clamp.on();
  chassis.drive_distance(-16);
  Intake.spin(reverse);
  // va chercher l'autre but
  chassis.drive_to_point(-5,13,0,10,12,2,200,3000,0.5,0.013,1.2,4,0.18,0.005,1.6,30);//0.2,0,0,0 // 15
 // chassis.drive_to_pose(0,15,270,0.5,1,0,12,12,2,200,3000,0.5,0.013,1.2,4,0.2,0,0,0);
 // chassis.drive_to_point(-4,17,0,12,12,2,200,3000,0.5,0.013,1.2,4,0.2,0,0,0);
  chassis.turn_to_angle(90);
  chassis.drive_distance(-22);
  Clamp.off();
  Intake.spin(forward);

  // va chercher le premier ring et deuxieme lautre bord

  chassis.turn_to_angle(10);
  chassis.drive_distance(20);

  chassis.turn_to_angle(270);
  chassis.drive_distance(27);
  chassis.turn_to_angle(346);
  chassis.drive_distance(24);
  chassis.drive_distance(-25);

  // va chercher le troisieme et 4ieme disque

  chassis.turn_to_angle(180);
  chassis.drive_distance(29,180,6,12);
  wait(0.3,seconds);

  // va chercher le 5 eme disque
  chassis.drive_distance(-12);
  Intake.spin(reverse);
  chassis.turn_to_angle(270);
  Intake.spin(forward);
  chassis.drive_distance(7);
  wait(0.3,seconds);

  // va chercher le mettre dans le coin

  chassis.turn_to_angle(15);
  Clamp.on();
  chassis.drive_distance(-15);
  Intake.spin(reverse);

  ///// le reste

  chassis.turn_to_angle(5);

  chassis.drive_distance(45);
  chassis.turn_to_angle(40);
  task testintae(IntakeUntilDisk);
  chassis.drive_distance(50);
  chassis.turn_to_angle(223);

  // va prendre le but

  chassis.drive_distance(-23);
  Clamp.off();

  // va chercher le ring au millieux
  chassis.drive_distance(34);
  chassis.turn_to_angle(120);
  double waittime = 0.6;
  task take2(IntakeButWait, (void *)&waittime);
  chassis.drive_distance(30);
  chassis.turn_to_angle(305);

  // va chercher le 3ieme ring 

  chassis.drive_distance(34);
  chassis.turn_to_angle(270);
  chassis.drive_distance(29);

  task intakedisck(IntakeUntilStop);

  // va chercher le 4ieme et 5ieme disque

  chassis.turn_to_angle(0);
  chassis.drive_distance(24,0,12,12);
  chassis.drive_distance(-6,0,12,12);
  chassis.turn_to_angle(20);
  chassis.drive_distance(18);
  /*chassis.drive_max_voltage = 12;
  Intake.stop();
  chassis.drive_distance(-12,0,12,12);
  Intake.spin(forward);
  chassis.drive_distance(12,0,12,12);
  Intake.stop();
  chassis.turn_to_angle(30);
  Intake.spin(reverse);
  chassis.drive_distance(12);
  chassis.drive_distance(-12);
  Intake.spin(forward);
  chassis.turn_to_angle(0);
  chassis.drive_distance(20);*/
  // va chercher le 6ieme dsque


  // va le mettre dans le coin

  chassis.turn_to_angle(125);
  Clamp.on();
  chassis.drive_timeout = 700;
  chassis.drive_distance(-22);
  chassis.drive_timeout = 2500;

  // va chercher le alliance stake
// StopIntake();
  //Intake.stop();
  Intake.spin(reverse);
  chassis.drive_distance(17);
  chassis.turn_to_angle(90);
  chassis.drive_distance(43);
  StopIntake();
  chassis.turn_to_angle(135);
  Intake.stop();

  task intakeuntilxs(IntakeUntilDisk);
  chassis.drive_distance(34);

  //chassis.turn_to_angle(150);
  chassis.drive_distance(-38);
  Clamp.off();
  chassis.turn_to_angle(180);
  chassis.drive_timeout = 800;
  chassis.drive_distance(-30,180,6,12);
  Intake.spin(forward);
  wait(0.3,seconds);
  chassis.drive_timeout = 2000;

  // va mettre le but dans le bo magnifique coin raph arrete de regarder pls tu me gosse vraiment beaucoup je vais écrire un paragraphe si tu continue a laide
// math 84 en fucking uautonome bruh

  chassis.turn_to_angle(290);
  Climb.set(!Climb.value());
  chassis.drive_distance(-100,260);
  chassis.drive_distance(20);
  chassis.drive_distance(60,205,6,12);






  

  // va vers lautre bord pour faire le high stakes.
  /*Intake.stop();
  task IntakeUnti(IntakeUntilDisk);
 // chassis.drive_to_point(-50,65,0,10,12,2,200,3000,0.5,0.013,1.2,4,0.2,0,0,0);
  //chassis.turn_to_angle(0);
  chassis.drive_to_pose(-50,65,0,0.4,0.2,0,12,12,2,200,3000,0.5,0.013,1.2,4,0.2,0,4,0);
  chassis.drive_distance(15);
  Clamp.off();
  //chassis.drive_to_point(-50,64,0,12,12,2,200,3000,0.5,0.013,1.2,4,0.2,0,0,0);
  //chassis.drive_to_pose(-20,45,45,0.5,1,12,12,12,2,200,3000,0.5,0.013,1.2,4,0.2,0,0,0);

  //chassis.drive_to_pose(-20,45,45,0.5,1,12,12,12,2,200,3000,0.5,0.013,1.2,4,0.2,0,0,0);
  chassis.drive_distance(-21,0);
  chassis.turn_to_angle(90);
  //chassis.drive_to_pose(-55,62,270,0.5,1,0,12,12,2,200,3000,0.5,0.013,1.2,4,0.2,0,0,0);
  chassis.drive_timeout = 1000;
  Lift.off();
  chassis.drive_distance(-27);
  Intake.spin(forward);
  wait(0.5,seconds);
  Intake.stop();
  chassis.drive_settle_time = 2000;

  // va chercher lautre but 
  Lift.on();*/

/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
  /*task UntilDisk(IntakeUntilDisk);
  chassis.AngleReversed = false;
  chassis.drive_to_point(-19,94,0,12,12,2.3,200,3000,0.7,0,7,3,0.5,0.0,4,10); // 76
  chassis.drive_distance(12,37); //14
  chassis.turn_to_angle(240);
  chassis.drive_distance(-20,215);
  Clamp.off();
  Intake.spin(forward);
  wait(0.5,seconds);

  // va chercher le premier disque du coin bleu
  chassis.drive_to_pose(-55,92,270,0.5,1,0,12,12);
  chassis.drive_settle_time = 50;
  chassis.drive_distance(-10,0);
  chassis.turn_to_angle(0);
  chassis.drive_distance(26,0);
  Intake.stop();
  chassis.drive_distance(15,75);
  Intake.spin(forward);
  chassis.drive_distance(-20,0);
  chassis.drive_timeout = 1600;
  chassis.drive_to_pose(chassis.get_X_position() + 6,chassis.get_Y_position() + 20,0,0.5,1,0,12,12);
  chassis.drive_distance(-9,0);
  chassis.turn_to_angle(270);
  chassis.drive_distance(12,270);
  chassis.turn_to_angle(160);
  Clamp.on();
  chassis.drive_distance(-20,160,12,12,2.5,300,1000);
  chassis.drive_timeout = 3000;
  Intake.stop();
  Intake.spinFor(reverse,0.2,seconds);*/

 // chassis.drive_to_point(-56,82);
  
  /*chassis.turn_to_angle(0);
  chassis.drive_distance(20);
  chassis.drive_distance(26,90);
  chassis.drive_distance(-35,0);
  chassis.drive_distance(18,90);*/

  // va le mettre dans le coin 
 /* wait(0.3,seconds);
  chassis.turn_to_angle(120);
  Clamp.on();
  Intake.spin(reverse);
  // va chercher prendre le ring et le mets sur le alliance stake
  chassis.drive_timeout = 3000;
  chassis.drive_distance(110,80); 
  chassis.AngleReversed = false;*/
  //Climb.on();
  //chassis.drive_to_point(30,70,70,12,12,2.3,200,3000,0.7,0,7,3,0.5,0.0,4,10); // 76
  //chassis.drive_to_point(0,63,70,12,12,2.3,200,3000,0.7,0,7,3,0.5,0.0,4,10); // 76
  //chassis.drive_to_pose(-45,75,240,0.5,1,0,12,12,2,200,3000,0.5,0.013,1.2,4,0.2,0,0,0);
  
}

void RushRougeProv()
{
  SetTeam(1);

  chassis.set_drive_constants(12,0.5,0.05,1.8,4);
  chassis.set_turn_constants(12,0.18,0.005,1.6,30); // percent = 7.4
  chassis.set_heading_constants(12,0.24,0.001,4.3,4);

  chassis.set_heading(180);
  chassis.drive_settle_time = 0;
  chassis.drive_distance(-46,205);
  Clamp.off();
  chassis.drive_distance(20,200);
  double waittime = 0.5;
  task tak3e(IntakeButWait, (void *)&waittime);
  chassis.turn_to_angle(46);
  chassis.drive_distance(15);

  // deuxieme but

  chassis.turn_to_angle(180);
  Clamp.on();
  wait(0.3,seconds);
  chassis.turn_to_angle(80);
  chassis.drive_distance(-28);
  Clamp.off();

  // 3ieme ring

  chassis.turn_to_angle(220);
  task Intaecolor(IntakeUntilDisk);
  chassis.drive_distance(12);
  chassis.turn_to_angle(270);
  chassis.drive_distance(36);
  

  // 4ieme ring

  chassis.turn_to_angle(290);
  chassis.drive_distance(23);
  wait(0.3,seconds);

}

void RushBleuProv()
{
  SetTeam(1);

  chassis.set_drive_constants(12,0.5,0.05,1.8,4);
  chassis.set_turn_constants(12,0.2,0.005,1.6,30); // percent = 7.4 P = 0.18
  chassis.set_heading_constants(12,0.24,0.001,4.3,4);

  chassis.set_heading(180);
  chassis.drive_settle_time = 0;
  //chassis.turn_settle_time = 100;
  chassis.drive_distance(-45.5,160);
  Clamp.off();
  wait(0.1,seconds);
  chassis.drive_distance(20,160);
  double waittime = 0.5;
  task tak3e(IntakeButWait, (void *)&waittime);
  chassis.turn_to_angle(314);
  chassis.drive_distance(15);

  // deuxieme but

  chassis.turn_to_angle(180);
  Clamp.on();
  wait(0.3,seconds);
  chassis.turn_to_angle(285);
  chassis.drive_distance(-28);
  Clamp.off();

  // 3ieme ring

  chassis.turn_to_angle(180);
  //task Intaecolor(IntakeUntilDisk);
  Intake.stop();
  Intake.spin(reverse);
  chassis.drive_distance(32);
  chassis.turn_settle_time = 100;
  Climb.off();
  chassis.turn_to_angle(98);
  chassis.drive_distance(67);
  

  // 4ieme ring

  chassis.turn_to_angle(0);
  Intake.spin(forward);
  chassis.drive_distance(48);

  chassis.drive_distance(-24,90);

}

// alligné au milieux du robot coin de la tuile sur le standoff ligné

void DroiteSaboRouge3()
{
  SetTeam(1);
  chassis.set_heading(0);
  chassis.set_drive_constants(12,0.5,0.05,1.8,4);
  chassis.set_turn_constants(12,0.18,0.005,1.6,30); // percent = 7.4
  chassis.set_heading_constants(12,0.24,0.001,4.3,4);

  task Intakeuntidks(IntakeUntilDisk);

  chassis.drive_distance(43);
  chassis.drive_distance(-22,270);
  Clamp.off();

  //chassis.turn_to_angle(320);
  task IntakeUntilStop532(IntakeUntilStop);
  chassis.turn_to_angle(205);
  chassis.drive_distance(30);
  chassis.turn_to_angle(240);
  chassis.drive_timeout = 1500;
  chassis.drive_distance(24);
  chassis.drive_timeout = 2500;
  chassis.turn_to_angle(210);
  chassis.drive_distance(-50);

}


void DroiteProvBleuREAL(){

  SetTeam(0);
  chassis.set_drive_constants(12,0.5,0.05,1.8,4);
  chassis.set_turn_constants(12,0.2,0.005,1.6,30); // percent = 7.4 P = 0.18
  chassis.set_heading_constants(12,0.24,0.001,4.3,4);
  chassis.set_heading(159);
  chassis.drive_distance(-20);
  Clamp.off();
  chassis.turn_to_angle(30);
  Intake.spin(forward);
  chassis.drive_to_pose(16,26.7,103,0.5,0.5,0,12,12,2,200,3000,0.5,0.013,1.5,4,0.2,0,4,0);

  chassis.drive_distance(-25,30);
  Clamp.on();


  // 3ieme ring

  chassis.turn_to_angle(103);
  Intake.stop();
  task awaitdisk(IntakeUntilDisk);
  chassis.drive_distance(12);

  // stake

  chassis.turn_to_angle(184);
  Climb.off();
  chassis.drive_to_pose(-37,3,282,0.5,0.5,0,12,12,2,200,3000,0.5,0.013,1.5,4,0.2,0,4,0);

  // but

  chassis.drive_distance(12,0);
  chassis.turn_to_angle(184);

  chassis.drive_distance(-20);
  Clamp.off();
  chassis.turn_to_angle(282);
  Intake.spin(forward);
  chassis.drive_distance(20);
  chassis.drive_distance(-36,220);

  /*chassis.turn_to_angle(184);
  chassis.drive_distance(34);
  Climb.off();
  chassis.turn_to_angle(278);
  chassis.drive_distance(60,270);*/


  //0.4,0.2,0,12,12,2,200,3000,0.5,0.013,1.5,4,0.2,0,4,0

};
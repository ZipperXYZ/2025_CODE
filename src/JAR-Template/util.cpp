#include "vex.h"
//#include "util.h"

/**
 * Converts an angle to an equivalent one in the range [0, 360).
 * 
 * @param angle The angle to be reduced in degrees.
 * @return Reduced angle.
 */

float reduce_0_to_360(float angle) {
  while(!(angle >= 0 && angle < 360)) {
    if( angle < 0 ) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return(angle);
}

/**
 * Converts an angle to an equivalent one in the range [-180, 180).
 * 
 * @param angle The angle to be reduced in degrees.
 * @return Reduced angle.
 */

float reduce_negative_180_to_180(float angle) {
  while(!(angle >= -180 && angle < 180)) {
    if( angle < -180 ) { angle += 360; }
    if(angle >= 180) { angle -= 360; }
  }
  return(angle);
}

/**
 * Converts an angle to an equivalent one in the range [-90, 90).
 * If the angle has no equivalent, then the angle halfway around
 * the circle is returned.
 * 
 * @param angle The angle to be reduced in degrees.
 * @return Reduced angle.
 */

float reduce_negative_90_to_90(float angle) {
  while(!(angle >= -90 && angle < 90)) {
    if( angle < -90 ) { angle += 180; }
    if(angle >= 90) { angle -= 180; }
  }
  return(angle);
}

/**
 * Converts an angle in degrees to radians.
 * 
 * @param angle The angle in degrees.
 * @return Angle in radians.
 */

float to_rad(float angle_deg){
  return(angle_deg/(180.0/M_PI));
}

/**
 * Converts an angle in radians to degrees.
 * 
 * @param angle The angle in radians.
 * @return Angle in degrees.
 */

float to_deg(float angle_rad){
  return(angle_rad*(180.0/M_PI));
}

/**
 * "Clamps" a number between a min and max.
 * Does no error checking and assumes min is less
 * than or equal to max.
 * 
 * @param input The number to be clamped.
 * @param min Minimum edge of the clamp.
 * @param max Maximum edge of the clamp.
 * @return Clamped number.
 */

float clamp(float input, float min, float max){
  if( input > max ){ return(max); }
  if(input < min){ return(min); }
  return(input);
}

/**
 * Checks if an input number corresponds to a reversed drive port.
 * 
 * @param input The number to be checked.
 * @return Whether the input is negative.
 */

bool is_reversed(double input){
  if(input<0) return(true);
  return(false);
}

/**
 * Scales a joystick to drive voltage scale.
 * Values get multiplied by 12 because motors can
 * output a max of 12 volts.
 * 
 * @param percent The input joystick reading.
 * @return The equivalent value in volts.
 */

float to_volt(float percent){
  return(percent*12.0/100.0);
}

/**
 * Converts an input port to a triport.
 * Does error checking to prevent invalid values from being
 * passed to the triport array.
 * 
 * @param port Input integer port.
 * @return Port-1, to convert it to a triport.
 */

int to_port(int port){
  if(port > 8 || port < 1){
    return(0);
  }
  return(port-1);
}

/**
 * Deadband function for joystick control.
 * If the joystick value is small, we should just consider it 0.
 * 
 * @param input The input joystick value.
 * @param width Minimum value to not get zeroed out.
 * @return The deadbanded value.
 */

float deadband(float input, float width){
  if (fabs(input)<width){
    return(0);
  }
  return(input);
}

/**
 * Settling control for odometry functions.
 * Draws a line perpendicular to the line from the robot to the desired 
 * endpoint, and checks if the robot has crossed that line. Allows for
 * very quick settling, and thereby chaining for fast motion control.
 * 
 * @param desired_X The ending X position in inches.
 * @param desired_Y The ending Y position in inches.
 * @param desired_angle_deg The direction of the line to be drawn.
 * @param current_X The robot's X position in inches.
 * @param current_Y The robot's Y position in inches.
 * @return Whether the robot can be considered settled.
 */

bool is_line_settled(float desired_X, float desired_Y, float desired_angle_deg, float current_X, float current_Y){
  return( (desired_Y-current_Y) * cos(to_rad(desired_angle_deg)) <= -(desired_X-current_X) * sin(to_rad(desired_angle_deg)) );
}

/**
 * Voltage scaling to keep from applying more than 12 volts to either side of the drive.
 * Divides both drive and heading output proportionally to get a similar result to the
 * desired one.
 * 
 * @param drive_output The forward output of the drive.
 * @param heading_output The angular output of the drive.
 * @return The scaled voltage for the left side of the robot.
 */

float left_voltage_scaling(float drive_output, float heading_output){
  float ratio = std::max(fabs(drive_output+heading_output), fabs(drive_output-heading_output))/12.0;
  if (ratio > 1) {
    return (drive_output+heading_output)/ratio;
  }
  return drive_output+heading_output;
}

/**
 * Voltage scaling to keep from applying more than 12 volts to either side of the drive.
 * Divides both drive and heading output proportionally to get a similar result to the
 * desired one.
 * 
 * @param drive_output The forward output of the drive.
 * @param heading_output The angular output of the drive.
 * @return The scaled voltage for the right side of the robot.
 */

float right_voltage_scaling(float drive_output, float heading_output){
  float ratio = std::max(fabs(drive_output+heading_output), fabs(drive_output-heading_output))/12.0;
  if (ratio > 1) {
    return (drive_output-heading_output)/ratio;
  }
  return drive_output-heading_output;
}

/**
 * Brings an output up to the minimum voltage if it's too slow.
 * Used for minimum voltage calculations for movement chaining.
 * Has no effect on 0 voltage output, because how do we know 
 * which way it's supposed to be going?
 * 
 * @param drive_output The forward output of the drive.
 * @param drive_min_voltage The minimum output of the drive.
 * @return The voltage with the minimum applied.
 */

float clamp_min_voltage(float drive_output, float drive_min_voltage){
  if(drive_output < 0 && drive_output > -drive_min_voltage){
      return -drive_min_voltage;
  }
  if(drive_output > 0 && drive_output < drive_min_voltage){
    return drive_min_voltage;
  }
  return drive_output;
}

float ReverseAngle(float angle){
  if (angle != 0){
    angle = 360 - angle;
    return angle;
  } else {
    return angle;
  }
}

bool IsBrasUp(){
  return !Lift.value();
}

bool Stop = false;

// team 1 = red team 0 = blue

int Team = 1;

int RedMinHUE = 7;
int RedMaxHUE = 16;

int BlueMinHUE = 200;
int BlueMaxHUE = 220;

int ChoosedMax;
int ChoosedMin;

int UnwantedMax;
int UnwantedMin;

float WaitTime = 0.09;//0.07;
float ReverseWaitTime = 0.1;
int MaxTime = 4500;

void SetTeam(int Nb){
  Team = Nb;
}

void StopIntake(){
  Stop = true;
}

void UnstopIntake(){
  Stop = false;
}

int IntakeUntilStop(){
  
  int Time = 0;

  if (Team == 1) {
    ChoosedMax = RedMaxHUE;
    ChoosedMin = RedMinHUE;
    UnwantedMax = BlueMaxHUE;
    UnwantedMin = BlueMinHUE;
  } else if (Team == 0) {
    ChoosedMax = BlueMaxHUE;
    ChoosedMin = BlueMinHUE;
    UnwantedMax = RedMaxHUE;
    UnwantedMin = RedMinHUE;
  }

  Intake.spin(forward);

  while (Stop != true)
  {
    if ((ColorSensor.hue() <= UnwantedMax && ColorSensor.hue() >= UnwantedMin)) {
      wait(WaitTime,seconds);
      Intake.spin(reverse);
      wait(ReverseWaitTime,seconds);
      Intake.spin(forward);
    }

    Intake.spin(forward);
    wait(20,msec);
  }

  Intake.stop();

  Stop = false;
  
  return 1;
}

int IntakeUntilDisk(){


  int Time = 0;

  if (Team == 1) {
    ChoosedMax = RedMaxHUE;
    ChoosedMin = RedMinHUE;
    UnwantedMax = BlueMaxHUE;
    UnwantedMin = BlueMinHUE;
  } else if (Team == 0) {
    ChoosedMax = BlueMaxHUE;
    ChoosedMin = BlueMinHUE;
    UnwantedMax = RedMaxHUE;
    UnwantedMin = RedMinHUE;
  }

  Brain.Screen.printAt(60,120,"running %f",ChoosedMax);

  while ((ColorSensor.hue() >= ChoosedMax || ColorSensor.hue() <= ChoosedMin) && Time < MaxTime)
  {
    if ((ColorSensor.hue() <= UnwantedMax && ColorSensor.hue() >= UnwantedMin)) {
      wait(WaitTime,seconds);
      Intake.spin(reverse);
      wait(ReverseWaitTime,seconds);
      Intake.spin(forward);
    }

    Intake.spin(forward);
    Time += 20;
    wait(20,msec);
  }

  Intake.stop();
  
  return 1;
}

int IntakeUntilDiskButWait(void *arg){
  
  double *timewait = (double *)arg;
  wait(*timewait,seconds);

  int Time = 0;

  if (Team == 1) {
    ChoosedMax = RedMaxHUE;
    ChoosedMin = RedMinHUE;
    UnwantedMax = BlueMaxHUE;
    UnwantedMin = BlueMinHUE;
  } else if (Team == 0) {
    ChoosedMax = BlueMaxHUE;
    ChoosedMin = BlueMinHUE;
    UnwantedMax = RedMaxHUE;
    UnwantedMin = RedMinHUE;
  }

  Brain.Screen.printAt(60,120,"running %f",ChoosedMax);

  while ((ColorSensor.hue() >= ChoosedMax || ColorSensor.hue() <= ChoosedMin) && Time < MaxTime)
  {
    if ((ColorSensor.hue() <= UnwantedMax && ColorSensor.hue() >= UnwantedMin)) {
      wait(WaitTime,seconds);
      Intake.spin(reverse);
      wait(ReverseWaitTime,seconds);
      Intake.spin(forward);
    }

    Intake.spin(forward);
    Time += 20;
    wait(20,msec);
  }

  Intake.stop();
  
  return 1;
}
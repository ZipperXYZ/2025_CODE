using namespace vex;

extern brain Brain;

// VEXcode devices
extern smartdrive Drivetrain;
extern motor ClawMotor;
extern motor ArmMotor;

extern motor leftMotorA;
extern motor leftMotorB;
extern motor leftMotorC;
extern motor rightMotorA;
extern motor rightMotorB;
extern motor rightMotorC;

//extern motor_group LeftDriveSmart;
//extern motor_group RightDriveSmart;
//extern Drive_train DriveX;

//extern inertial Inertial2;
extern inertial Inertial2;
//extern rotation FowardEncoder;
//extern rotation SideEncoder;
extern controller Controller1;

/*extern motor Intake;
extern motor MoteurBras;
S
extern digital_out PneumaBras;
extern digital_out PneumaBut;*/

extern led intake;
extern led Lift;
extern led Clamp;

extern motor Intake_moteur;
extern motor MoteurBras;
extern motor_group Intake;


/**
 * Used to initialize code/tasks/sdevices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
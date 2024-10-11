#include "vex.h"
#include "autons.cpp"

using namespace vex;

// A global instance of competition
competition Competition;

//robot config.cpp stuff
motor LF = motor(PORT8, ratio18_1, true);
motor LM = motor(PORT10, ratio18_1, true);
motor LB = motor(PORT2, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(LF, LM, LB);
motor RF = motor(PORT3, ratio18_1, false);
motor RM = motor(PORT15, ratio18_1, false);
motor RB = motor(PORT1, ratio18_1, false);
motor_group RightDriveSmart = motor_group(RF, RM, RB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
inertial Inertial1 = inertial(PORT7);
rotation tracking1 = rotation(PORT20);
rotation tracking2 = rotation(PORT19);

#include "vex.h"
using namespace vex;
using signature = vision::signature;
using code = vision::code;
brain  Brain;
controller Controller1 = controller(primary);
motor intake = motor(PORT9, ratio6_1, false);
// Pistons
digital_out Clamp = digital_out(Brain.ThreeWirePort.H);
digital_out PTO = digital_out(Brain.ThreeWirePort.B);
digital_out elevated_intake = digital_out(Brain.ThreeWirePort.G);
digital_out elevation = digital_out(Brain.ThreeWirePort.F);

void PID_turn(double LeftVelocity,double RightVelocity,double inches_traveled) {
  LeftDriveSmart.setVelocity(LeftVelocity, pct);
  RightDriveSmart.setVelocity(RightVelocity,pct);
  Drivetrain.driveFor(inches_traveled, inches,false);
  wait(1,sec);
  Drivetrain.stop();
}

void Reset_Both_Sides(double same_velocity) {
  LeftDriveSmart.setVelocity(same_velocity, pct);
  RightDriveSmart.setVelocity(same_velocity,pct);
}

void win_point_red() {
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-14,inches);
  Drivetrain.turnFor(45,degrees);
  Reset_Both_Sides(30);
  Drivetrain.driveFor(-8,inches);
  Drivetrain.driveFor(0,inches,false);
  intake.spin(forward);
  Clamp.set(true);
  Drivetrain.turnFor(-90,degrees);
  Drivetrain.driveFor(12,inches);
  wait (4, sec);
  PID_turn(0,100,-3);
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-16,inches);
}

void left_red() {
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-15,inches);
  Reset_Both_Sides(20);
  Drivetrain.driveFor(-4,inches);
  Clamp.set(true);
  wait(1,sec);
  intake.spin(forward);
  Drivetrain.turnFor(58,degrees);
  Drivetrain.driveFor(11,inches);
  wait (1, sec);
  intake.spin(reverse);
  Drivetrain.driveFor(-13,inches);
  intake.stop();
  Drivetrain.turnFor(27,degrees);
  intake.spin(forward);
  Drivetrain.driveFor(10,inches);
  PID_turn(0,100,7);
  Reset_Both_Sides(100);
  wait (1,sec);
  Drivetrain.driveFor(10,inches);
  Drivetrain.turnFor(-27,degrees);
  intake.spinFor(-50,degrees);
  Drivetrain.turnFor(27,degrees);
  intake.spin(forward);
  Drivetrain.driveFor(5,inches);
}

void right_blue() {
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-15,inches);
  Reset_Both_Sides(20);
  Drivetrain.driveFor(-4,inches);
  Clamp.set(true);
  wait(1,sec);
  intake.spin(forward);
  Drivetrain.turnFor(-58,degrees);
  Drivetrain.driveFor(11,inches);
  wait (1, sec);
  intake.spin(reverse);
  Drivetrain.driveFor(-13,inches);
  intake.stop();
  Drivetrain.turnFor(-27,degrees);
  intake.spin(forward);
  Drivetrain.driveFor(-10,inches);
  PID_turn(100,0,7);
  Reset_Both_Sides(100);
  wait (1,sec);
  Drivetrain.driveFor(10,inches);
  Drivetrain.turnFor(27,degrees);
  intake.spinFor(-50,degrees);
  Drivetrain.turnFor(-27,degrees);
  intake.spin(forward);
  Drivetrain.driveFor(5,inches);
}

void skills_auton() {
  Inertial1.calibrate();
  Reset_Both_Sides(30);
  intake.spinFor(1500,degrees);
  Drivetrain.driveFor(8,inches);
  Drivetrain.turnFor(55,degrees);
  Reset_Both_Sides(30);
  Drivetrain.driveFor(-10,inches);
  Drivetrain.driveFor(-1,inches,false);
  Clamp.set(true);
  Reset_Both_Sides(50);
  Drivetrain.turnFor(-62,degrees);
  //delay to avoid a wheely
  wait(0.05,sec);
  Reset_Both_Sides(40);
  intake.spin(forward);
  Drivetrain.driveFor(10.9,inches);
  //this is the most inconsitent value
  PID_turn(0,50,10.5);
  //
  Reset_Both_Sides(50);
  wait (1,sec);
  Drivetrain.driveFor(9.5,inches);
  wait (1,sec);
  Drivetrain.turnFor(-49,degrees);
  Drivetrain.driveFor(10.8,inches);
  wait (2,sec);
  Drivetrain.driveFor(6,inches);
  wait (1,sec);
  //
  Reset_Both_Sides(40);
  Drivetrain.driveFor(-5.5,inches);
  Drivetrain.turnFor(63,degrees);
  Drivetrain.driveFor(5,inches);
  Reset_Both_Sides(30);
  wait (1,sec);
  Drivetrain.turnFor(54,degrees);
  Reset_Both_Sides(40);
  Drivetrain.driveFor(24,inches);
  Reset_Both_Sides(30);
  wait (3,sec);
  Drivetrain.driveFor(-31,inches);
  Drivetrain.turnFor(25,degrees);
  Drivetrain.driveFor(-3,inches);
  Clamp.set(false);
  intake.stop();
  Drivetrain.driveFor(9,inches);
  Drivetrain.turnFor(-83,degrees);
  Reset_Both_Sides(30);
  Drivetrain.driveFor(10.5,inches,false);
  wait (2,sec);
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-38,inches);
  Reset_Both_Sides(30);
  Drivetrain.driveFor(-7,inches);
  //other side
  Clamp.set(true);
  Reset_Both_Sides(50);
  Drivetrain.turnFor(62,degrees);
  //delay to avoid a wheely
  wait(0.05,sec);
  Reset_Both_Sides(40);
  intake.spin(forward);
  Drivetrain.driveFor(10.5,inches);
  //this is the most inconsitent value
  PID_turn(50,0,10.5);
  //
  Reset_Both_Sides(50);
  wait (1,sec);
  Drivetrain.driveFor(9.5,inches);
  wait (1,sec);
  Drivetrain.turnFor(52,degrees);
  Drivetrain.driveFor(10.8,inches);
  wait (2,sec);
  Drivetrain.driveFor(6,inches);
  wait (1,sec);
  //
  Reset_Both_Sides(40);
  Drivetrain.driveFor(-5.5,inches);
  Drivetrain.turnFor(-63,degrees);
  Drivetrain.driveFor(5,inches);
  Drivetrain.turnFor(-63,degrees);
  Drivetrain.driveFor(-5,inches);
  Clamp.set(false);
  Drivetrain.turnFor(-33,degrees);
  elevation.set(true);
  wait(1,sec);
  Reset_Both_Sides(100);
  Drivetrain.driveFor(40,inches);
}

void win_point_blue() {
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-14,inches);
  Drivetrain.turnFor(-45,degrees);
  Reset_Both_Sides(30);
  Drivetrain.driveFor(-8,inches);
  Drivetrain.driveFor(0,inches,false);
  intake.spin(forward);
  Clamp.set(true);
  Drivetrain.turnFor(90,degrees);
  Drivetrain.driveFor(12,inches);
  wait (4, sec);
  PID_turn(100,0,-3);
  Reset_Both_Sides(50);
  Drivetrain.driveFor(-16,inches);
}

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      
      int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position();

      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}


//robot config.cpp stuff



bool Clamping = false;
bool elevate = false;
bool tier_1 = false;

//settings:
double kp = 0.0;
double ki = 0.0;
double kd = 0.0;
double turnkp = 0.0;
double turnki = 0.0;
double turnkd = 0.0;

//autonomous settings
int desiredValue = 200;
int desiredTurnValue = 0;

int Error; 
//sensor value - desired value : position
int PrevError = 0; 
//position 20 miliseconds ago
int Derivative; 
//Error - PrevError : speed
int TotalError; 
//TotalError = TotalError + Error

int turnError;
int turnPrevError = 0;
int turnDerivative;
int turnTotalError;

bool ResetDriveSensors = true;

//varibles modified for use:
bool enablePID_Drive = true;
int PID_Drive (){

while (enablePID_Drive) {
  if (ResetDriveSensors) {
    ResetDriveSensors = false;
    LeftDriveSmart.setPosition(0,degrees);
    RightDriveSmart.setPosition(0,degrees);
  }

  //get the position of both motors
  int leftMotorPosition = LeftDriveSmart.position(degrees);
  int rightMotorPosition = RightDriveSmart.position(degrees);
  

  ////////////////////////////////////////////////////////////////////////////////
  //lateral movement PID
  ///////////////////////////////////////////////////////////////////////////////

  //get average of the two motors
  int averagePosition = (leftMotorPosition + rightMotorPosition)/2;

    //potential
    Error = averagePosition - desiredValue;

    //derivative
    Error = Error - PrevError;

    //velocity -> position -> absement
    TotalError += Error;

    //integral
    TotalError += Error;

    double lateralMotorPower = Error * kp + Derivative * kd + TotalError * ki;

   ////////////////////////////////////////////////////////////////////////////////



  ////////////////////////////////////////////////////////////////////////////////
  //turning movement PID
  ///////////////////////////////////////////////////////////////////////////////
  //get average of the two motors
  int turnDifference = (leftMotorPosition + rightMotorPosition)/2;

    //potential
    turnError = turnDifference - desiredTurnValue;

    //derivative
    turnDerivative = turnError - turnPrevError;

    //velocity -> position -> absement
    TotalError += Error;

    //integral
    //turnTotalError += turnError

    double turnMotorPower = turnError * turnkp + turnDerivative * turnkd;


  ////////////////////////////////////////////////////////////////////////////////

    LeftDriveSmart.spin(forward, lateralMotorPower + turnMotorPower, velocityUnits::pct);
    RightDriveSmart.spin(forward, lateralMotorPower + turnMotorPower, velocityUnits::pct);





  PrevError = Error;
  turnPrevError = Error;
}

return 1;
}


int auton = 0;
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Drivetrain.setDriveVelocity(100,percent);
  intake.setVelocity(100, percent);
  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(0, 0, 200,117);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(0, 117, 200,117);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(300, 0, 200,117);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(300, 117, 200,117);
  Brain.Screen.setFillColor(ClrPurple);
  Brain.Screen.drawRectangle(175, 0, 125,117);
  Brain.Screen.setFillColor(ClrPink);
  Brain.Screen.drawRectangle(175, 117, 125,117);
  Brain.Screen.setCursor(3,3);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("win point blue");
  Brain.Screen.setCursor(9,3);
  Brain.Screen.setFillColor(red);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("Left Red");
  Brain.Screen.setCursor(3,36);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("Right Blue");
  Brain.Screen.setCursor(9,36);
  Brain.Screen.setFillColor(red);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("win point Red");
  Brain.Screen.setCursor(3,22);
  Brain.Screen.setFillColor(ClrPurple);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("Skills");
  Brain.Screen.setCursor(9,20);
  Brain.Screen.setFillColor(ClrPink);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("Calibrate");
  //touch screen
  while (true){
    if (Brain.Screen.pressing()){
     if (Brain.Screen.xPosition()< 180){
       if (Brain.Screen.yPosition()<120){
        auton = 1;
         Brain.Screen.setCursor(1,1);
         Brain.Screen.setFillColor(blue);
         Brain.Screen.print("Win point Blue  ");
       }
     } 
     if (Brain.Screen.xPosition()> 180){
       if (Brain.Screen.yPosition()> 120){
         auton = 6;
         Brain.Screen.setCursor(1,1);
         Brain.Screen.setFillColor(blue);
         Brain.Screen.print("Calibrating      ");
         Inertial1.calibrate();
         tracking1.setPosition(0,degrees);
         tracking2.setPosition(0,degrees);
         wait (1, sec);
         Brain.Screen.setCursor(1,1);
         Brain.Screen.print("3               ");
         wait (1, sec);
         Brain.Screen.setCursor(1,1);
         Brain.Screen.print("2               ");
         wait (1, sec);
         Brain.Screen.setCursor(1,1);
         Brain.Screen.print("1               ");
         wait (1, sec);
         Brain.Screen.setCursor(1,1);
         Brain.Screen.print("Calibrated      ");
       }
     } 
     if (Brain.Screen.xPosition()> 300){
       if (Brain.Screen.yPosition()>120){
         auton = 4;
         Brain.Screen.setCursor(1,1);
         Brain.Screen.setFillColor(blue);
         Brain.Screen.print("Win point Red  ");
       }
     } 
     if (Brain.Screen.xPosition()< 180){
       if (Brain.Screen.yPosition()>120){
         auton = 2;
         Brain.Screen.setCursor(1,1);
         Brain.Screen.setFillColor(blue);
         Brain.Screen.print("Left Red       ");
       }
     }
     if (Brain.Screen.xPosition()> 180){
       if (Brain.Screen.yPosition()< 120){
         auton = 5;
         Brain.Screen.setCursor(1,1);
         Brain.Screen.setFillColor(blue);
         Brain.Screen.print("Skills         ");
       }
     }  
     if (Brain.Screen.xPosition()> 300){
       if (Brain.Screen.yPosition()< 120){
         auton = 3;
         Brain.Screen.setCursor(1,1);
         Brain.Screen.setFillColor(blue);
         Brain.Screen.print("Right Blue     ");
       }
     }  
    }
    wait(20, msec);
  }
}

void autonomous(void) {
  if (auton == 1) {
    win_point_blue();
  }
  else if (auton == 2) {
    left_red();
  }
  else if (auton == 3) {
    right_blue();
  }
  else if (auton == 4) {
    win_point_red();
  }
  else if (auton == 5) {
    skills_auton();
  }
  else {
    win_point_blue();
  }
  
  /*vex::task DrivetrainPID(PID_Drive);

  desiredValue = 300;
  desiredTurnValue = 600;

  vex::task::sleep(1000);

  ResetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 300;*/
} 

void usercontrol(void) {
  //enablePID_Drive = false;
  // User control code here, inside the loop
  while (1) {
    intake.setVelocity(100, pct);
    Drivetrain.setDriveVelocity(100, pct);

    // Clamp toggle
    if (Controller1.ButtonY.pressing()) {
      if (Clamping == false){
        Clamping = true;
        wait(.1, sec);        
      }
       else if (Clamping == true){
        Clamping = false;
        wait(.1, sec);
      }
    }
    Clamp.set(Clamping);

    // Intake Elevation Toggle    
  if (Controller1.ButtonRight.pressing()) {
      if (elevate == false){
        elevate = true;
        wait(.1, sec);
      }
       else if (elevate == true){
        elevate = false;
        wait(.1, sec);
      }
    }
    elevated_intake.set(elevate);

    // Elevation Toggle
  if (Controller1.ButtonDown.pressing()) {
      if (tier_1 == false){
        tier_1 = true;
        wait(.1, sec);
      }
       else if (tier_1 == true){
        tier_1 = false;
        wait(.1, sec);
      }
    }
    elevation.set(tier_1);

    // Intake Spinner
    if (Controller1.ButtonL1.pressing()) {
      intake.spin(forward);
    }else if (Controller1.ButtonL2.pressing()) {
      intake.spin(reverse);
    }else if (true){
      intake.stop();
    }
  LeftDriveSmart.setVelocity(100, pct);
  RightDriveSmart.setVelocity(100,pct);
  


    wait(.1, sec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.

  }
}



//kasen did not miss a semicolon - mckay

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
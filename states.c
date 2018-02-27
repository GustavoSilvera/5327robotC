#pragma config(Sensor, in1,    FourBarPot,     sensorPotentiometer)
#pragma config(Sensor, in2,    LiftPot,        sensorPotentiometer)
#pragma config(Sensor, in3,    MoGoPot,        sensorPotentiometer)
#pragma config(Sensor, in5,    Gyro,           sensorGyro)
#pragma config(Sensor, in6,    BATERY_2_PORT,  sensorAnalog)
#pragma config(Sensor, dgtl1,  RightEncoder,   sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  LeftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl6,  GoliathEnc,     sensorRotation)
#pragma config(Sensor, dgtl11, OddLED,         sensorLEDtoVCC)
#pragma config(Sensor, dgtl12, EvenLED,        sensorLEDtoVCC)
#pragma config(Motor,  port1,           goliath,       tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           LiftTop,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           Base_R_F,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           Base_L_F,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           Base_R_M,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           Base_R_B,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           Base_L_M,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           Base_L_B,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           LiftBottom,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          Bar,           tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// This code is for the VEX cortex platform
#pragma platform(VEX2)
#pragma competitionControl(Competition)

#include "Vex_Competition_Includes.c"
#include "include/states/randomstuff.h"
#include "include/states/initialize.h"
#include "include/states/sensor.h"
#include "include/states/lift.h"
#include "include/states/base.h"
#include "include/states/LCD.h"
#include "include/states/auton.h"

#define NONE 100
#define rev true
//extern
struct PIDs;
struct liftMech;
struct sideBase;


void initializeOpControl(const bool driver) {
	SensorValue[EvenLED] = 0;
	SensorValue[OddLED] = 0;
	clearLCDLine(0);
	clearLCDLine(1);
	if(driver) resetGyros();//wastes time on auton
		resetEncoders();
	velocity = 0.0;

	//-LIFT---------&reference--TYPE----------sensor-1-----motor-1-----motor-2-------max------min----delay(opt)
	initLiftType(   &mainLift,  NORMAL,       LiftPot,     LiftTop,    LiftBottom,   4400,    2150           );
	initLiftType(   &mogo,      DIFFERENTIAL, LiftPot,     LiftTop,    LiftBottom,   9000,    -9000          );//is this max (4000) supposed to be that off?
	initLiftType(   &FourBar,   BINARY,       FourBarPot,  Bar,        NONE,         3640,    1900,  10      );

	//-PID------&reference------sensor--------------thresh--kP------kI------kD------reversed----running(opt)----
	initPID(    &mainLift.PID,  mainLift.sensor,    50,     0.15,   0.0,    0.0,    rev,        true          );
	initPID(    &FourBar.PID,   FourBar.sensor,     250,    0.2,    0.0,    0.0,    rev,        false         );
	initPID(    &gyroBase,      Gyro,               3,      0.525,  0.0,    0.5,    !rev,       false         );

	//-SIDE---------&reference----sensor------------motor-1------motor-2--------motor-3------
	initSideBase(   &Left,        LeftEncoder,      Base_L_F,    Base_L_M,      Base_L_B   );
	initSideBase(   &Right,       RightEncoder,     Base_R_F,    Base_R_M,      Base_R_B   );
	initSideBase(   &goliat,      GoliathEnc,  		goliath,     NONE,          NONE       );
	pastRot = mRot;
}

void pre_auton() {//dont care
	bStopTasksBetweenModes = true;
	SensorType[Gyro] = sensorGyro;
	wait1Msec(1224);
	//Adjust SensorScale to correct the scaling for your gyro
	scaleGyros();
	startTask(displayLCD);
	while( bIfiRobotDisabled ){//in preauton...bIfiRobotDisabled ||
		autonSelect(nLCDButtons);
		delay(50);
	}
}
task mogoOut(){
	autonRunning = true;
	intakeSpeed = INTAKE;
	FourBar.goal = FourBar.min;
	UpUntil(&mainLift, SensorValue[mainLift.sensor] + 450);
	FourBar.goal = FourBar.max;
	clearTimer(T4);
	while(SensorValue[mogo.sensor] > mogo.min && time1[T4] < 600){
		liftDiff(&mogo, 127);
	}
	return;
}
void fourConeAuton(bool right, bool twenty){
	int dir  = 1;
	if(!right) dir = -1;
	autonRunning = true;
	mainLift.PID.kP = 0.15;
	startTask(mogoOut);
	FourBar.PID.isRunning = true;
	intakeSpeed = INTAKE/2;
	driveFor2(48);//48
	//FourBar.goal = FourBar.max;
	stopTask(mogoOut);
	fwds(0);
	clearTimer(T4);
	while(time1[T4] < 600){
		FourBar.goal = FourBar.max;
		liftDiff(&mogo, -127);
	}
	liftMove(&mogo, 0);
	intakeSpeed = OUTTAKE; //release preload
	delay(400);
	intakeSpeed = 0;
	driveFor(1);
	FourBar.goal = FourBar.min;
	DownUntil(&mainLift, mainLift.min, 127); //go for next cone
	mainLift.goal = mainLift.min - 200;
	mainLift.PID.isRunning = true;
	intakeSpeed = INTAKE; //intake first cone
	//driveFor(2);
	delay(350);
	stack(2); //stack first cone
	FourBar.goal = FourBar.min;
	mainLift.goal = mainLift.min + 600;
	driveFor(1);
	//driveFor(-1);
	DownUntil(&mainLift, mainLift.min, 80);
	mainLift.goal = mainLift.min-400;
	mainLift.PID.isRunning = true;
	intakeSpeed = INTAKE;
	delay(400);
	stack(3);//stack second cone
	FourBar.goal = FourBar.min - 200;
	intakeSpeed = INTAKE;
	DownUntil(&mainLift, mainLift.min, 80);
	mainLift.goal = mainLift.min-400;
	mainLift.PID.isRunning = true;
	driveFor(1);
	delay(700);

	/*driveFor(2);
	//driveFor(-1);
	DownUntil(&mainLift, mainLift.min, 80);
	mainLift.goal = mainLift.min-400;
	mainLift.PID.isRunning = true;
	intakeSpeed = INTAKE;
	delay(300);*/
	stack(4);
	intakeSpeed = 0;

	//score in 10pt

	if(!twenty){ //score in 10pt
		driveFor2(-62);//drive back
		mainLift.goal = mainLift.max; //lift lift
		mainLift.PID.isRunning = true;
		if (right) RSwingFor(-45);//swing out
		else LSwingFor(45);
		rotFor(dir * -90, 2); //rotate perp to 10pt pole
		delay(200);
		driveFor(4); //drive to 10pt
		//mainLift.PID.isRunning = false;
		clearTimer(T4);
		while(SensorValue[mogo.sensor] > mogo.min && time1[T4] < 600){ //mogo out
			liftDiff(&mogo, 127);
		}
		//mainLift.PID.isRunning = true;
		driveFor(-25);// release & get out of the way
	}
	else { //score in 20 pt
		driveFor2(-62);//drive back
		mainLift.goal = mainLift.max; //lift lift
		mainLift.PID.isRunning = true;
		if(right) RSwingFor(-45);
		else LSwingFor(45);
		driveFor(-6); //drive to center of zone
		rotFor(dir * -90,2);
		driveFor(-5);//reverse to gain momentum
		delay(100);
		fwds(127);
		delay(1500);
		fwds(0);
		//driveFor(16); //enter 20pt zone
		clearTimer(T4);
		while(SensorValue[mogo.sensor] > mogo.min + 100 && time1[T4] < 600){ //mogo out
			liftDiff(&mogo, 127);
		}
		driveFor(-17);//exit zones
	}
	autonRunning = false;
}
void matchLoadAuton (bool right, bool twenty){
	int dir  = 1;
	if(!right) dir = -1;
	autonRunning = true;
	mainLift.PID.kP = 0.15;
	startTask(mogoOut);
	FourBar.PID.isRunning = true;
	intakeSpeed = INTAKE/2;
	driveFor(50);//48
	//FourBar.goal = FourBar.max;
	stopTask(mogoOut);
	fwds(0);
	clearTimer(T4);
	while(time1[T4] < 600){
		FourBar.goal = FourBar.max;
		liftDiff(&mogo, -127);
	}
	intakeSpeed = OUTTAKE; //release preload
	driveFor(-45);
	intakeSpeed = 0;
	rotFor(45);

	}
void stagoAuton (bool right){

}
task autonomous() {
	autonRunning = true;
	initializeOpControl(false);//auton init
	startTask(LiftControlTask);//individual pid for lift type
	startTask(MeasureSpeed);//velocity measurer for base
	startTask(autoStack);
	startTask(antiStall);
	startTask(killswitch);
	if(slewRating)startTask(MotorSlewRateTask);
	startTask(displayLCD);
	startTask(goliathControl);
	FourBar.PID.kP = 0.9;
	FourBar.PID.thresh = 30;
	switch( currentAutonomous ) {
	case    0:
		fourConeAuton(RIGHT, TEN);
		break;
	case    1:
		fourConeAuton(LEFT, TWENTY);
		break;
	case    2:
		fourConeAuton(RIGHT, TEN);
		break;
	case    3:
		fourConeAuton(LEFT, TEN);
		break;
	case    4:

		break;

	case    5://no auton
		break;
	}
	autonRunning = false;
	return;
}
task usercontrol() {//initializes everything
	initializeOpControl(true);//driver init
	startTask(LiftControlTask);//individual pid for lift type
	startTask(MeasureSpeed);//velocity measurer for base
	startTask(autoStack);
	startTask(antiStall);
	startTask(killswitch);
	if(slewRating)startTask(MotorSlewRateTask);
	startTask(goliathControl);
	startTask(displayLCD);
	autonRunning = false;
	bLCDBacklight = true;// Turn on LCD Backlight
	clearLCDLine(0); // Clear line 1 (0) of the LCD
	clearLCDLine(1); // Clear line 2 (1) of the LCD
	if(nImmediateBatteryLevel < 8000) playSound(soundException);
	else playSound(soundUpwardTones);
	for (;;) {
		//debug controls
		//if (U7) fourConeAuton(RIGHT, TEN);//matchLoadAuton(RIGHT, TEN);//threeConeAuton(LEFT);//rotFor(-10);
		//	if (R7) driveFor2(20);//fourConeAuton(RIGHT, TWENTY);
		//	if (L7) driveFor2(20);//fourConeAuton(RIGHT, TWENTY);
		//if (D7) //fourConeAuton(RIGHT, TEN);
		driveCtrlr();
		delay(15);//~60hz
	}
}//function for operator control

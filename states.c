#pragma config(Sensor, in1,    FourBarPot,     sensorPotentiometer)
#pragma config(Sensor, in2,    LiftPot,        sensorPotentiometer)
#pragma config(Sensor, in5,    Gyro,           sensorGyro)
#pragma config(Sensor, in3,    MoGoPot,        sensorPotentiometer)
#pragma config(Sensor, in6,    BATERY_2_PORT,  sensorAnalog)
#pragma config(Sensor, dgtl1,  LeftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  RightEncoder,   sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  ultraSound,     sensorSONAR_cm)
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

	//-LIFT---------&reference--TYPE------------sensor-1--------motor-1-----motor-2-----max------min----delay(opt)
	initLiftType(	&mainLift,	NORMAL,			LiftPot,				LiftTop,	LiftBottom, 4020,	 2150	);
	initLiftType(	&mogo,		DIFFERENTIAL,	LiftPot,				LiftTop,	LiftBottom, 4000,	600 );
	initLiftType(	&FourBar,	BINARY,	 		FourBarPot,			Bar,		NONE,		3830,	 2000,	10);
	initLiftType(	&goliat,		NOPID,	 		NONE,					goliath,	NONE,		10000,	 -10000	);//(TUNE)

	//-PID------&reference------sensor--------------thresh--kP------kI------kD------reversed----running(opt)
	initPID(	&mainLift.PID,	mainLift.sensor,			10,	  0.25,	 0.0,	   0.0, 	rev,	true);
	initPID(	&FourBar.PID, 	FourBar.sensor,	    	30, 	  0.95, 	 0.0,   	0.0,   rev, 	false);
	initPID (&gyroBase, 		Gyro, 						12,  	  0.525,  0.0, 	0.5, 	!rev, 	false);

	//-SIDE---------&reference----sensor------------motor-1------motor-2--------motor-3
	initSideBase( 	&Left, 		  LeftEncoder, 		Base_L_F, 	 Base_L_M,		Base_L_B);
	initSideBase( 	&Right, 	 	  RightEncoder,		Base_R_F, 	 Base_R_M,		Base_R_B);
	pastRot = mRot;
}

void pre_auton() {//dont care
	bStopTasksBetweenModes = true;
	SensorType[Gyro] = sensorGyro;
	wait1Msec(1224);
	//Adjust SensorScale to correct the scaling for your gyro
	scaleGyros();
	while( bIfiRobotDisabled ){//in preauton...bIfiRobotDisabled ||
		autonSelect(nLCDButtons);
		delay(50);
	}
}

task autonomous() {
	autonRunning = true;
	initializeOpControl(false);//auton init
	startTask(LiftControlTask);//individual pid for lift type
	startTask(MeasureSpeed);//velocity measurer for base
	startTask(sensorsUpdate);
	startTask(antiStall);
	switch( currentAutonomous ) {
	case    0:
		threeConeAuton(RIGHT);
		break;
	case    1:
		threeConeAuton(LEFT);
		break;
	case    2:
		EZAuton(RIGHT);
		break;
	case    3:
		EZAuton(LEFT);
		break;
	case    4:
		kamakaze();
		break;

	case    5://no auton
		break;
	}
	return;
}
task mogoOut(){

	UpUntil(&mainLift, SensorValue[mainLift.sensor] + 150);
	clearTimer(T4);
	while(SensorValue[mogo.sensor] > mogo.min && time1[T4] < 600){
		liftDiff(&mogo, 127);
	}
	return;
}
void auton(bool right){
	int dir  = 1;
	if(!right) dir = -1;
	startTask(goliathHold);
	startTask(mogoOut);
	FourBar.goal = FourBar.max;
	FourBar.PID.isRunning = true;
	intakeSpeed = INTAKE/2;
	driveFor(55);
	fwds(0);
	clearTimer(T4);
	while(time1[T4] < 600){
		liftDiff(&mogo, -127);
	}
	intakeSpeed = OUTTAKE;
	delay(400);

	stopTask(goliathHold);
	stopTask(mogoOut);


}
task usercontrol() {//initializes everything
	initializeOpControl(true);//driver init
	startTask(LiftControlTask);//individual pid for lift type
	startTask(MeasureSpeed);//velocity measurer for base
	startTask(sensorsUpdate);
	startTask(autoStack);
	startTask(antiStall);
	startTask(killswitch);
	startTask(displayLCD);
	autonRunning = false;
	bLCDBacklight = true;// Turn on LCD Backlight
	clearLCDLine(0); // Clear line 1 (0) of the LCD
	clearLCDLine(1); // Clear line 2 (1) of the LCD
	if(nImmediateBatteryLevel < 8000) playSound(soundException);
	else playSound(soundUpwardTones);
	if(SensorValue[ultraSound] < 1){//0 or error
		playSound(soundLowBuzz);//sonar error (CRITICAL)
	}
	for (;;) {
		//debug controls
		if (L7) rotFor(-90);//threeConeAuton(LEFT);//rotFor(-10);
		if (R7) rotFor(90);
		if(D7) auton(RIGHT);
		driveCtrlr();
		delay(15);//~60hz
	}
}//function for operator control

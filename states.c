#pragma config(Sensor, in1,    FourBarPot,     sensorPotentiometer)
#pragma config(Sensor, in2,    LiftPot,        sensorPotentiometer)
#pragma config(Sensor, in4,    Gyro,           sensorGyro)
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
	initLiftType(	&mainLift,	NORMAL,			LiftPot,				LiftTop,	LiftBottom, 4020,	 2000	);
	initLiftType(	&mogo,		DIFFERENTIAL,	LiftPot,				LiftTop,	LiftBottom, 4050,	 0 );
	initLiftType(	&FourBar,	NORMAL,	 		FourBarPot,			Bar,		NONE,		3700,	 1850,	10);
	initLiftType(	&goliat,		NOPID,	 		NONE,					goliath,	NONE,		10000,	 -10000	);//(TUNE)

	//-PID------&reference------sensor--------------thresh--kP------kI------kD------reversed----running(opt)
	initPID(	&mainLift.PID,	mainLift.sensor,	30,	    0.45,	0.0,	0.05, 	rev,	true);
	initPID(	&FourBar.PID, 	FourBar.sensor,	    10, 	0.35, 	0.0,   	0.01,   rev, 	true);
	//initPID ( 	&gyroBase, 		Gyro, 				0,  	0.525, 	0.0, 	0.5, 	reversed, 	false);

	//-SIDE---------&reference----sensor------------motor-1------motor-2--------motor-3
	initSideBase( 	&Left, 		  LeftEncoder, 		Base_L_F, 	 Base_L_M,		Base_L_B);
	initSideBase( 	&Right, 	  RightEncoder,		Base_R_F, 	 Base_R_M,		Base_R_B);
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
	if(nImmediateBatteryLevel < 8500) playSound(soundException);
	else playSound(soundUpwardTones);
	if(SensorValue[ultraSound] < 1){//0 or error
		playSound(soundLowBuzz);//sonar error (CRITICAL)
	}
	for (;;) {
		//debug controls
		if (L7 || L7_2 )threeConeAuton(LEFT);//rotFor(-10);
		driveCtrlr();
		delay(15);//~60hz
	}
}//function for operator control

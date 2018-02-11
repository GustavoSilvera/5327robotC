#pragma config(Sensor, in1,    RLin,           sensorLineFollower)
#pragma config(Sensor, in2,    LLin,           sensorLineFollower)
#pragma config(Sensor, in3,    LockPot,        sensorPotentiometer)
#pragma config(Sensor, in4,    FourBarPot,     sensorPotentiometer)
#pragma config(Sensor, in5,    BATERY_2_PORT,  sensorAnalog)
#pragma config(Sensor, in6,    Gyro,           sensorGyro)
#pragma config(Sensor, in7,    Gyro2,          sensorGyro)
#pragma config(Sensor, dgtl1,  LeftBaseEnc,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  RightBaseEnc,   sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  MogoFront,      sensorTouch)
#pragma config(Sensor, dgtl6,  MogoEnd,        sensorTouch)
#pragma config(Sensor, dgtl8,  sonar,          sensorSONAR_cm)
#pragma config(Motor,  port2,           ClawMotor,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           RBaseFront,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           RBaseBack,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           RFourBarMotor, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           ConveyorMotor, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           LBaseFront,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           LBaseBack,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           LFourBarMotor, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          LockMotor,     tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
// This code is for the VEX cortex platform
#pragma platform(VEX2)
// Select Download method as "competition"
#pragma competitionControl(Competition)
//Main competition background code...do not modify!5
#include "Vex_Competition_Includes.c"
#include "include/randomstuff.h"
#include "include/dataStructures.h"
#include "include/mech.h"
#include "include/rotate.h"
#include "include/physics.h"
#include "include/lift.h"
#include "include/init.h"
#include "include/other.h"

void pre_auton() {//dont care
	bStopTasksBetweenModes = true;
	SensorType[Gyro] = sensorGyro;
	wait1Msec(1220);
	//Adjust SensorScale to correct the scaling for your gyro
	scaleGyros();
	while (bIfiRobotDisabled) {//in preauton
		startTask(displayLCD);
	}
}
void initializeOpControl(const bool driver) {
	clearLCDLine(0);
	clearLCDLine(1);
	if (driver) resetGyros();//wastes time on auton
		mainVelocity = 0.0;
	//--------&reference-------TYPE-----------sensor-----------motor1---------motor2------------motor3-----(min, max)
	initMech( &conveyer,	 CONVEYER,    0,		ConveyorMotor,		0 	);//CONVEYOR
	initMech( &baseLeft,	 DRIVE,	      	LeftBaseEnc,		LBaseFront,		  LBaseBack);//LEFT BASE
	initMech( &baseRight,	 DRIVE,		  	RightBaseEnc,		RBaseFront,		  RBaseBack);//RIGHT BASE
	initLift( &lock,								LockPot,			LockMotor, 			0,						1000, 2000);//(min) && (max)
	initLift( &fourBar,							FourBarPot,		RFourBarMotor,		LFourBarMotor,		1300,	3500, 10);
	//--------&reference-------Sensor------thresh---kP---kI---kD---reversed---isRunning
	initPID ( &fourBar.PID, fourBar.m.sensor, 50, 0.05,  0.0, 0.05, 	true, 		true);
	initPID ( &lock.PID,		lock.m.sensor, 		30, 0.5, 	 0.0, 0.0, 		true, 		true);
	initPID ( &gyroBase, 		Gyro, 						1,  1, 0.0, 0, 		true, 		false);//kP = .35, kD = 0.6
	pastRot = mRot;
}
task intakeToLock(){
	while(SensorValue[MogoEnd] == 0){
		mechMove(&conveyer.m, INTAKE);
	}
	//delay(250);
	lock.goal = lock.min;//turns on PID
	lock.PID.isRunning = true;
	playSound(soundBeepBeep);
	LOCK;//brings up levitator
	lock.goal = SensorValue[lock.m.sensor];//turns on PID
	lock.PID.isRunning = true;
	//mechMove(&conveyer.m, 0);
	return;
}
task intakeSecond(){
	//pick up second mogo
	while(SensorValue[MogoFront] == 0){
		mechMove(&conveyer.m, INTAKE);
	}
	delay(100);
	clawControl(OPEN);
	while(SensorValue[MogoFront] == 1){
		mechMove(&conveyer.m, INTAKE);//slower speed once intook (button pressed)
	}
	//clawControl(OPEN);clawControl(OPEN);clawControl(OPEN);clawControl(OPEN);//spams claw open just to make sure
	playSound(soundBlip);
	mechMove(&conveyer.m, 0);
	return;
}
task dropCone(){
	while(SensorValue[MogoFront] == 0)//wait until button pressed
		continue;
	playSound(soundUpwardTones);
	clawControl(OPEN);//spams claw open to ensure
	delay(1000);
	fourBar.goal = fourBar.min + 400;//then brings out the fourbar a bit
	fourBar.PID.isRunning = true;
	return;
}
task conveyerMove(){
	stopTask(MechControlTask);
	for(;;){
		if(autonRunning) mechMove(&conveyer.m, conveyer.speed);//allows multitasking
			delay(50);
	}
}
volatile int clawSpeed = 0;
task clawHold(){
	for(;;){
		motor[ClawMotor] = clawSpeed;
		delay(30);
	}
}
void twentyScore(){
	lock.PID.isRunning = true;
	conveyer.speed = 0;
	lock.goal = lock.max;//unlocks mogo
	fwds(-127, mRot);
	delay(1000);
	conveyer.speed = INTAKE;
	fwds(-127, mRot);
	delay(400);
	fwds(0, mRot);
	delay(400);
}
void crossField(const int initialDistance, const int secondaryDistance, bool withCone) {
	const int initialMRot = SensorValue[Gyro]*GyroK;
	lock.goal = lock.max;
	UNLOCK;
	conveyer.speed = INTAKE;
	fourBar.goal = RELEASE + 500;
	fourBar.PID.isRunning = true;
	startTask(intakeToLock);//picks up first mogo
	driveFor(initialDistance);
	fourBar.goal = RELEASE;
	fwds(70, mRot);//slows down drive
	clearTimer(T1);
	if(withCone){
		while(abs(SensorValue[Gyro]*GyroK - initialMRot) > 1 && time1[T1] < 500)
			rot(-10*(SensorValue[Gyro]*GyroK - initialMRot));//corrects angle if off course
	}
	else{
		while(SensorValue[Gyro]*GyroK > initialMRot && time1[T1] < 500)
			rot(-10*(SensorValue[Gyro]*GyroK - initialMRot));//corrects angle if off course
	}
	delay(500);//waits for mogo pickup
	startTask(intakeSecond);
	if (withCone) startTask(dropCone);
	driveFor(40);//drive initial distance to get to cone-free wall
	if (withCone) alignSonar(40, 4);//aligns to the wall using the sonar... should be p consistent (TUNE)
	else alignSonar(35, 4);
	//driveFor(secondaryDistance);
	fourBar.goal = avg(fourBar.min, fourBar.max);
	conveyer.speed = INTAKE / 6;
	delay(300);
	lock.goal = lock.min;
	settle();
}
void Thoar(){
	const int initialMRot = SensorValue[Gyro]*GyroK;
	settle();
	startTask(intakeToLock);
	driveFor(90);
	clearTimer(T1);
	//while(SensorValue[Gyro]*GyroK > initialMRot && time1[T1] < 700)
	//	rot(-10*(SensorValue[Gyro]*GyroK - initialMRot));
	settle();
	driveFor(-10);
	fourBar.goal = PICKUP;
	delay(200);
	clawSpeed = -127;
	startTask(clawHold);
	delay(200);
	while(SensorValue[Gyro] > -25) rot(-127);
	settle();
	driveFor(13);
	clawSpeed = 40;//pickup cone
	delay(250);
	stopTask(clawHold);
	delay(100);//settling
	DownUntil(&fourBar, fourBar.min, 90);
	while(SensorValue[Gyro] > -55) rot(-127);
	//delay(000);
	settle();
	driveFor(23);
	delay(100);
	startTask(intakeSecond);
	delay(300);
	startTask(dropCone);
	startTask(clawHold);
	fwds(127);
	delay(800);
	driveFor(-5);
	settle();
	delay(400);
	startTask(clawHold);
	clawSpeed = -127;
	delay(500);
	driveFor(-10);
	clawSpeed = 80;
	stopTask(clawHold);
	delay(100);
	fourBar.goal = PICKUP;
	conveyer.speed = 0;
	delay(400);
	rotAcc(-50, 15);
}
void progSkills(){
	startTask(conveyerMove);
	startTask(killswitch);
	startTask(liftPID);
	autonRunning = true;
	startTask(intakeToLock);
	driveFor(90);
	delay(500);
	driveFor(-80);
	rotAcc(45, 5);
	int goal = 32.5;
	while(SensorValue[sonar] < goal){
		fwds(4*(SensorValue[sonar] - goal));
	}
	rotAcc(-90, 5);
	UNLOCK;
	conveyer.speed = INTAKE;
	driveFor(-15);
	delay(200);
	///////////////////////////////STAGE 1/////////////////////////////
	crossField(80, 70, true);//second distance dosent do anything (TUNE) (using sonar for 2nd distance)
	//conveyer.speed = 0;
	delay(100);
	rotAcc(-90, 5);
	//timeTurn(90, 2);//time based turn with 2 mogos @ 90�
	fourBar.goal = RELEASE;
	delay(300);
	alignSonar(68, 4);
	settle();
	delay(100);
	rotFor(-90);
	//timeTurn(90, 2);//time based turn with 2 mogos @ 90�
	fourBar.goal = avg(fourBar.min, fourBar.max);
	delay(400);
	twentyScore();//SCORES FIRST TWO IN ZONE
	settle();
	delay(100);
	fwds(127);
	delay(450);//time based drive to go over 10pt pole but not pass line (TUNE)
	//for ^ could also use driveFor(25) ish, need to (TUNE) regardless
	fourBar.goal = RELEASE;//brings to rear again
	//'s' motion
	alignToLine(1);//aligns to tape line
	conveyer.speed = 0;//turn off conveyer
	settle();
	delay(200);
	rotAcc(90, 5, 900);//should be solid 90 every time
	settle();
	alignSonar(42, 5.5);
	settle();
	delay(300);
	rotAcc(-89, 5, 1100);
	settle();
	///////////////////////////////STAGE 2/////////////////////////////
	crossField(80, 62, false);
	fourBar.goal = RELEASE;
	conveyer.speed = 0;
	rotAcc(-90, 10);
	//timeTurn(90, 2);//time based turn with 2 mogos @ 90�
	alignSonar(64, 4);
	settle();
	rotAcc(-90, 10);
	//timeTurn(90, 2);//time based turn with 2 mogos @ 90�
	twentyScore();
	fwds(127);
	delay(600);//time based drive to go over 10pt pole but not pass line (TUNE)

	///////////////////////////////STAGE 3/////////////////////////////
	//alignToLine(1);
	//rotAcc(90, 5);//should be accurate 90 every time
	//alignSonar(25, 6);
	//settle();
	//rotAcc(-45, 2);
//	Thoar();//
//	driveFor(50);
//	alignToLine(1.5);
//	rotAcc(-90, 5);
//	alignSonar(90, 4);
//	rotFast(-90);
//	UNLOCK;
//	conveyer.speed = INTAKE;
////	driveFor(-15);
////	settle();
//	delay(400);
//	driveFor(15);
////	alignToLine(1);
	//DONE
	autonRunning = false;
	startTask(MechControlTask);
	return;
}
task autonomous() {
	autonRunning = true;
	initializeOpControl(false);//auton init
	startTask(MechControlTask);//individual pid for lift type
	startTask(MeasureSpeed);//velocity measurer for base
	startTask(sensorsUpdate);
	startTask(conveyerMove);//allows for multitasking
	startTask(clawTask);
	progSkills();
	return;
}
task usercontrol() {//initializes everything
	initializeOpControl(true);//driver init
	startTask(MechControlTask);//individual pid for lift type
	startTask(MeasureSpeed);//velocity measurer for base
	startTask(sensorsUpdate);
	startTask(antiStall);
	//startTask(killswitch);
	startTask(displayLCD);
	startTask(clawTask);
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	autonRunning = false;
	fourBar.goal = RELEASE;
	if(nImmediateBatteryLevel < 8300) playSound(soundException);
	else playSound(soundUpwardTones);
	for (;;) {
		//debug controls
		//if(D7) rotFast(45);//timeTurn(90, 2);//(TUNED)
		//if(R7) rotFast(-45);
		if(U7) progSkills();
		if(L7) progSkillsTestFIRST();
		LiftLift(&fourBar, D5, U5, D5_2, U5_2, 5000, true);
		LiftLift(&lock, U8, D8, U8_2, D8_2, 100, true);
		delay(30);//~60hz
	}
}//function for operator control

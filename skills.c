#pragma config(Sensor, in1,    RLin,           sensorNone)
#pragma config(Sensor, in2,    LLin,           sensorLineFollower)
#pragma config(Sensor, in3,    MLin,           sensorLineFollower)
#pragma config(Sensor, in4,    MFLin,          sensorLineFollower)
#pragma config(Sensor, in5,    BATERY_2_PORT,  sensorAnalog)
#pragma config(Sensor, in6,    Gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  LeftBaseEnc,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  RightBaseEnc,   sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  MogoFront,      sensorTouch)
#pragma config(Sensor, dgtl6,  MogoEnd,        sensorTouch)
#pragma config(Sensor, dgtl8,  Lsonar,         sensorSONAR_cm)
#pragma config(Sensor, dgtl11, Rsonar,         sensorSONAR_cm)
#pragma config(Motor,  port2,           RConveyor,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           RBaseFront,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           RBaseBack,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           elevator,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           intake,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           LBaseFront,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           LBaseBack,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           LConveyor,     tmotorVex393_MC29, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// This code is for the VEX cortex platform
#pragma platform(VEX2)
// Select Download method as "competition"
#pragma competitionControl(Competition)
//Main competition background code...do not modify!5
#include "Vex_Competition_Includes.c"
#include "include/skills/randomstuff.h"
#include "include/skills/dataStructures.h"
#include "include/skills/mech.h"
#include "include/skills/PID.h"
#include "include/skills/lift.h"
#include "include/skills/other.h"
#include "include/skills/drive.h"
#include "include/skills/rotate.h"
#include "include/skills/physics.h"
#include "include/skills/init.h"

int startDir = 0;

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
	resetencoders();
	//--------&reference-------TYPE-----------sensor-----------motor1---------motor2-----//
	initMech( &conveyer,       CONVEY,        0,               LConveyor,     RConveyor);//CONVEYOR
	initMech( &baseLeft,       DRIVE,         LeftBaseEnc,     LBaseFront,    LBaseBack);//LEFT BASE
	initMech( &baseRight,      DRIVE,         RightBaseEnc,    RBaseFront,    RBaseBack);//RIGHT BASE
	//--------&reference-------Sensor------------thresh---kP----kI---kD-----reversed---isRunning
	initPID ( &gyroBase,       Gyro,             1,       1,    0.0, 0,     true,      false);//kP = .35, kD = 0.6
	pastRot = mRot;
}
task intakeMogos(){
	//pick up first mogo
	while(SensorValue[MFLin] > 350){//SensorValue[MogoFront] == 0){
		motor[LConveyor] = INTAKE;
		motor[RConveyor] = INTAKE;
	}
	playSound(soundBlip);
	motor[LConveyor] = 0;
	motor[RConveyor] = 0;
	delay(300);
	//pick up second mogo
	while(SensorValue[MLin] > 350){
		motor[LConveyor] = INTAKE;
		motor[RConveyor] = INTAKE;
	}
	while(SensorValue[MFLin] > 350){
		motor[LConveyor] = INTAKE;
		motor[RConveyor] = INTAKE;
	}
	playSound(soundBlip);
	delay(550); //finish intaking mogo
	motor[LConveyor] = 0;
	motor[RConveyor] = 0;
	return;
}
task intakeMogo(){
	//intake one mogo, all the way at the top
	while(SensorValue[MogoEnd] == 0){
		//keep running conveyer
		motor[LConveyor] = INTAKE;
		motor[RConveyor] = INTAKE;
	}
	while(SensorValue[MogoEnd] == 1){
		//keep running conveyer
		motor[LConveyor] = INTAKE;
		motor[RConveyor] = INTAKE;
	}
	playSound(soundBlip);
	motor[LConveyor] = 0;
	motor[RConveyor] = 0;
	return;
}
task conveyerMove(){
	stopTask(MechControlTask);
	for(;;){
		if(autonRunning) mechMove(&conveyer.m, conveyer.speed);//allows multitasking
			delay(50);
	}
}
void skillsPart1() {
	//startTask(conveyerMove);
//	startTask(killswitch);
	//Corner Mogo (LOADER)
	startDir = mRot;
	motor[LConveyor] = INTAKE;
	motor[RConveyor] = INTAKE;
	startTask(intakeMogo);
	driveFor(60);//drive and grab mogo
//	if( abs( startDir - mRot ) > 2) rotTune( startDir - mRot + 2);//angular correction
	driveFor(-58);//come back to 10pt pole
	RSwingFor(-40);//align with 10pt
	clearTimer(T4);
	while(time1[T4] < 400){//release mogo in 10pt OPTIMIZE
		motor[LConveyor] = INTAKE;
		motor[RConveyor] = INTAKE;
	}
	delay(300);
	motor[LConveyor] = 0;
	motor[RConveyor] = 0;

	//Cross Field
	/*
	startTask(intakeMogos);
	driveFor(110); //drive and intake 2 mogos
	rotEnc(130);*/

	//End
	//startTask(MechControlTask);
}
void skillsPart2(bool first){

	//Cross Field
	startTask(intakeMogos);
	driveFor(110); //intake mogos

	//Double Mogo Score
	if(first) rotEnc(125); //OPTIMIZE closer to left side of zone
	else rotEnc(114); //OPTIMIZE closer to left side of zone
	driveFor(-19); //drive to center
	LSwingFor(45); //swing to 10pt pole
	clearTimer(T4);
	while(time1(T4)<900) fwds(-127);//full power drive forward
	//driveFor(18);
	clearTimer(T4);
	while(time1(T4)<1000) {//outtake 20pt mogo
		motor[LConveyor] = INTAKE;
		motor[RConveyor] = INTAKE;
	}
	clearTimer(T4);
	while(time1(T4)<500) {//start driving out of 10pt
		motor[LConveyor] = INTAKE;
		motor[RConveyor] = INTAKE;
		fwds(127);
	}
	clearTimer(T4);//drive conveyor in case 2nd mogo doesn't outtake
	while(time1(T4)<100) {
		motor[LConveyor] = INTAKE;
		motor[RConveyor] = INTAKE;
	}

	//alignToLine(1);

	//End
	//startTask(MechControlTask);
}
void skillsPart3() {

	//Align after double mogo
	alignToLine(1);
	driveFor(8);
	rotEnc(-90);
	delay(200);

	//Right Mogo
	driveToSonar(RIGHT, 11, 3); //drive to wall
	RSwingFor(30); //face mogo
	startDir = mRot;
	//delay(200);
	startTask(intakeMogo);
	driveFor(12); //drive to middle line
	alignToLine(1); //align line
	driveFor(30); //intake mogo

	//driveFor(47);
	if( abs( startDir - mRot ) > 3) rotTune((startDir - mRot));//angular correction
	driveFor(-55); //drive back
	LSwingFor(47); //align with 10pt
	clearTimer(T4);
	while(time1[T4]<400){ //release mogo in 10pt
		motor[LConveyor] = INTAKE;
		motor[RConveyor] = INTAKE;
	}
	//delay(400);
	motor[LConveyor] = 0;
	motor[RConveyor] = 0;

	//Align after right mogo
	alignToLine(1);
	driveFor(7);
	rotEnc(90);

	//Left Mogo
	driveToSonar(LEFT, 11, 3); //drive to wall
	LSwingFor(-35); //face mogo
	delay(200);
	startTask(intakeMogo);
	//playSound(soundFastUpwardTones);
	driveFor(20);
	clearTimer(T4);
	//playSound(soundFastUpwardTones);
	while(time1[T4]<70) { rot(127); }
	rot(0);
	driveFor(27);
	//driveFor(-10);
	//clearTimer(T4);
	//while(time1[T4]<100){ rot(-127); }
	//playSound(soundFastUpwardTones)
	//rot(0);
	//driveFor(-35);
	driveFor(-48);
	//playSound(soundFastUpwardTones);
	RSwingFor(-40); //align with 10pt
	delay(300);
	clearTimer(T4);
	while(time1[T4]<400){
		motor[LConveyor] = INTAKE;
		motor[RConveyor] = INTAKE;
	}
	delay(400);//release mogo in 10pt
	motor[LConveyor] = 0;
	motor[RConveyor] = 0;

	//skillsPart2(); //repeat skills Part 2
}
void skillsPart4 () {
	//Align after double mogo
	alignToLine(1);
	driveFor(7);
	rotEnc(-90);
	delay(300);

	//Right Mogo
	driveToSonar(RIGHT, 8, 3); //drive to wall
	RSwingFor(30); //face mogo
	delay(100);
	startDir = mRot;
	startTask(intakeMogo);
	driveFor(15); //intake mogo
	alignToLine(1);
	driveFor(25);
	if( abs(startDir - mRot ) > 3) rotTune((startDir - mRot));//angular correction
	driveFor(-50); //drive back
	LSwingFor(47); //align with 10pt
	clearTimer(T4);
	while(time1[T4]<300){
		motor[LConveyor] = INTAKE;
		motor[RConveyor] = INTAKE;
	}
	delay(400);//release mogo in 10pt
	motor[LConveyor] = 0;
	motor[RConveyor] = 0;

	//Park
	LSwingFor(-45);
	driveFor(60);
}
void progSkills(){
	//skills order: 1, 2, 3, 2, 4
	skillsPart1();
	skillsPart2(true);
	skillsPart3();
	skillsPart2(false);
	skillsPart4();
	return;
}

task autonomous(){
	initializeOpControl(true);//driver init
	startTask(MechControlTask);//individual pid for lift type
	startTask(MeasureSpeed);//velocity measurer for base
	startTask(sensorsUpdate);
	//startTask(antiStall); no
	startTask(killswitch);
	startTask(displayLCD);
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	progSkills();
}
task usercontrol() {//initializes everything
	initializeOpControl(true);//driver init
	startTask(MechControlTask);//individual pid for lift type
	startTask(MeasureSpeed);//velocity measurer for base
	startTask(sensorsUpdate);
	//startTask(antiStall); no
	startTask(killswitch);
	startTask(displayLCD);
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	autonRunning = false;
	if(nImmediateBatteryLevel < 8500) playSound(soundException);
	else playSound(soundUpwardTones);

	for (;;) {
		//debug controls
		//do not use D7 -- killswitch
		if(L7) alignToLine(1);
		if(U7) driveFor(50);
		if(R7) driveFor(-45);
		if(D7) startTask(intakeMogos);
		if(R8) progSkills();
		if(U5) skillsPart1();
		if(D5) skillsPart2(true);
		if(U8) skillsPart3();
		if(D8) skillsPart4();

		//accelerometer driving:
		/*
		float yaxis = 2*vexRT[AccelY];//drive
		float xaxis = 2*vexRT[AccelX];//rotate
		if(abs(yaxis) > abs(xaxis)){
			analogMechControl(&baseRight.m, -yaxis );
			analogMechControl(&baseLeft.m, -yaxis );
		}
		else{
			analogMechControl(&baseRight.m, -xaxis );
			analogMechControl(&baseLeft.m,   xaxis );
		}
		//if(abs(yaxis) > 20) 			fwds(-TruSpeed(yaxis));
		//else if(abs(xaxis) > 20)	rot(-TruSpeed(xaxis));
		*/
		delay(30);//~60hz
	}
}//function for operator control

#pragma config(Sensor, in1,    LockPot,            sensorPotentiometer)
#pragma config(Sensor, in2,    RLin,           sensorLineFollower)
#pragma config(Sensor, in3,    LLin,           sensorLineFollower)
#pragma config(Sensor, in4,    TLin,           sensorLineFollower)
#pragma config(Sensor, in5,    BLin,           sensorLineFollower)
#pragma config(Sensor, in6,    Gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  LeftBaseEnc,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  RightBaseEnc,   sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  BtnMogo,        sensorTouch)
#pragma config(Motor,  port1,           LockMotor,     tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port3,           RFBase,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           RBBase,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           lock,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           conveyor,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           LFBase,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           LBBase,        tmotorVex393_MC29, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

int lineThresh = 1000;

float TruSpeed(float value) {//for all other polynomials; visit: goo.gl/mhvbx4
	return((value*value*value) / (127*127));
}

void fwds(int power) {
	motor[RFBase] = -power;
	motor[RBBase] = -power;
	motor[LFBase] = power;
	motor[LBBase] = power;
}
/*
void driveFor(float goal) {//drives for certain inches
	SensorValue[LeftBaseEnc] = 0;
	SensorValue[RightBaseEnc] = 0;
	goal *= 2;//doubles "goal" not tuned very well as of rn
	const int thresh = 5;//10 ticks
	const int initDir = mRot;
	//ClearTimer(T1);
	const float dP = 20;//multiplier for velocity controller
	float vel = mainVelocity;
	while (abs(goal * circum - encoderAvg*0.25) > thresh) {
		fwds(limitDownTo(15, dP * ((goal*circum - encoderAvg*0.25 - 0.1*vel))), initDir);
	}
	fwds(0, initDir);
	return;
}
*/
void rot(int power) {
	//negative: CW
	//positive: CCW
	motor[RFBase] = power;
	motor[RBBase] = power;
	motor[LFBase] = power;
	motor[LBBase] = power;
}

void swingL(int power) {
	motor[LFBase] = power;
	motor[LBBase] = power;
}

void swingR(int power) {
	motor[RFBase] = power;
	motor[RBBase] = power;
}

task vroomvroom(){
	int speed;
	for(;;){
		if (abs(vexRT[Ch3])>10) {
			speed = TruSpeed(vexRT[Ch3]);
			motor[LFBase] = speed;
			motor[LBBase] = speed;
		}
		else {
			motor[LFBase] = 0;
			motor[LBBase] = 0;
		}

		if (abs(vexRT[Ch2])>10) {
			speed = TruSpeed(vexRT[Ch2]);
			motor[RFBase] = -speed;
			motor[RBBase] = -speed;
		}
		else {
			motor[RFBase] = 0;
			motor[RBBase] = 0;
		}

		if (vexRT[Btn5U]) motor[lock] = 127;
		else if (vexRT[Btn5D])motor[lock] = -127;
		else motor[lock] = 0;

		if (vexRT[Btn6U]){
			motor[conveyor] = 127;
		}
		else if (vexRT[Btn6D]){
			motor[conveyor] = -127;
		}
		else {
			motor[conveyor] = 0;
		}
	}
}

void alignPerp(int dir){
	bool isAligned = (SensorValue[RLin] + SensorValue[LLin] < lineThresh);
	int power = 25;
	while(!isAligned){
		if(SensorValue[LLin] < lineThresh){
			fwds(0);
			while(SensorValue[RLin] > lineThresh){
				swingR(dir * power);
			}
			isAligned = true;
		}
		else if (SensorValue[RLin] < lineThresh){
			fwds(0);
			while(SensorValue[LLin] > lineThresh){
				swingL(dir * power);
			}
			isAligned = true;
		}
		else {
			fwds(dir * 25); //speed where robot will stop on line
		}
	}
	fwds(0);
}



task main()
{
	SensorFullCount[Gyro] = 36000;
	startTask(vroomvroom);
	for(;;){
		if(vexRT[Btn7U]){ //left swing fwd
			swingL(100);
			delay(100);
			fwds(0);
		}
		if(vexRT[Btn7L]){ //left swing bck
			swingL(-100);
			delay(100);
			fwds(0);
		}
		if(vexRT[Btn7R]){ //right swing fwd
			swingR(100);
			delay(100);
			fwds(0);
		}
		if(vexRT[Btn7D]){ //right swing bck
			swingR(-100);
			delay(100);
			fwds(0);
		}
		if(vexRT[Btn8R]){
			alignPerp(-1);
		}
		delay(10);
	}
}

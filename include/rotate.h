#include "include/PID.h"
#include "include/drive.h"

//extern void settle();
void rot(float power) {//rotates base
	motor[RBaseFront] = power;
	motor[RBaseBack] = power;
	motor[LBaseFront] = -power;
	motor[LBaseBack] = -power;
}
void timeTurn(int target, int numMogos){//(TUNE)
	rot(getSign(target)*127);
	if(numMogos == 2){
		switch(abs(target)){
		case 45:
			delay(270);
			break;
		case 90:
			delay(420);
			break;
		}
	}
	else if (numMogos == 0){
		switch(abs(target)){
		case 45:
			delay(500);
			break;
		case 90:
			delay(630);
			break;
		}
	}
	return;
}
void turn(int deg, int df){
	int speed = GETSIGN(deg) * 127;
	float initialRot = mRot*GyroK;

	motor[RBaseFront] = speed;
	motor[RBaseBack] = speed;
	motor[LBaseFront] = -speed;
	motor[LBaseBack] = -speed;
	switch(abs(deg)){
		case 45:
			delay(250);
			break;
		case 90:
			delay(400);
			break;
	}
	motor[RBaseFront] = 0;
	motor[RBaseBack] = 0;
	motor[LBaseFront] = 0;
	motor[LBaseBack] = 0;

	float error = (mRot - initialRot);
	while (abs(error - deg) < df) {
		speed = GETSIGN(error) * 75;
		motor[RBaseFront] = speed;
		motor[RBaseBack] = speed;
		motor[LBaseFront] = -speed;
		motor[LBaseBack] = -speed;
	}
}
void rotFor(float target){
	gyroBase.isRunning = true;
	SensorValue[Gyro2] = 0;//resets gyros
	SensorScale[Gyro2] = 260;
	clearTimer(T2);
	while(abs(SensorValue[Gyro2]*GyroK - target) > 3 && time1[T2] < 900){//2 dF
		rot( 0.35*pidController( & gyroBase, target/GyroK) );
	}
	int power;
	if(target < 0) power = 127;
	else power = 127;
	rot(getSign(mainVelocity) * power);//gives settle time
	delay(100);
	rot(0);
	gyroBase.isRunning = false;
	resetPIDVals(&gyroBase);
	settle();
	return;
}
void rotEnc(int target){
	SensorValue[LeftBaseEnc] = 0;///reset
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	delay(100);
	if(abs(target) == 90){
		while(abs(SensorValue[LeftBaseEnc]) < 250){
			rot(getSign(target)*127);
		}
		rot(-getSign(target)*127);
		delay(100);
		while(SensorValue[Gyro]*GyroK < target) rot(60);
		while(SensorValue[Gyro]*GyroK > target) rot(-60);
	}
}
void rotAcc(int target, int delayTime = 1200){
	SensorValue[LeftBaseEnc] = 0;///reset
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	clearTimer(T2);
	while(time1[T2] < delayTime){
		rot(LimitDownTo(10, -5*(SensorValue[Gyro]*GyroK - target)));
	}
	settle();
}

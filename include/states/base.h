#if !defined(BASE_H)
#define BASE_H
//function for driving the robot
/*******************************************************************************\
* .----------------.  .----------------.  .----------------.  .----------------. *
*| .--------------. || .--------------. || .--------------. || .--------------. |*
*| |   ______     | || |      __      | || |    _______   | || |  _________   | |*
*| |  |_   _ \    | || |     /  \     | || |   /  ___  |  | || | |_   ___  |  | |*
*| |    | |_) |   | || |    / /\ \    | || |  |  (__ \_|  | || |   | |_  \_|  | |*
*| |    |  __'.   | || |   / ____ \   | || |   '.___`-.   | || |   |  _|  _   | |*
*| |   _| |__) |  | || | _/ /    \ \_ | || |  |`\____) |  | || |  _| |___/ |  | |*
*| |  |_______/   | || ||____|  |____|| || |  |_______.'  | || | |_________|  | |*
*| |              | || |              | || |              | || |              | |*
*| '--------------' || '--------------' || '--------------' || '--------------' |*
* '----------------'  '----------------'  '----------------'  '----------------' *
\*******************************************************************************/
float TruSpeed(const float value, const float power) {//for all other polynomials; visit: goo.gl/mhvbx4
	float final = pow(value, power) / (pow(127, power - 1));
	//if(value % 2 == 0) return (int)sgn(value) * final;
	return final;
}//function for calculating the truSpeed function based off a polynomial
int motorSlew[8];//for each motor
int SlewAmount[8];//for each motor

task MotorSlewRateTask(){//slew rate task
	int motorI;//which motor are we talking about? INDEX
	for(int i = 0; i < 8; i++){
		SlewAmount[i] = 30;
	}
	for(;;){// run loop for every motor
		for( motorI = 0; motorI < 8; motorI++){
			//motorCurrent = motor[ motorIndex ];
			if( abs(motor[motorI] - motorSlew[motorI]) > 10){//current motor value not equal to goal
				if( motor[motorI] < motorSlew[motorI] && motor[motorI] < 127){//if less than goal || less than max
					motor[motorI] =  motor[motorI] + SlewAmount[motorI];//decrease by specific amount
					if( motor[motorI] >= motorSlew[motorI]){//if equal to or surpassed goal
						motor[motorI]  = motorSlew[motorI];//sets change to goal
					}
				}
				if( motor[motorI] > motorSlew[motorI] && motor[motorI] > -127){//if currently more than requested
					motor[motorI] = motor[motorI] - SlewAmount[motorI];//decrease by specific amount
					if(motor[motorI] <= motorSlew[motorI]){//once reaches or passes goal
						motor[motorI] = motorSlew[motorI];//sets change to goal
					}
				}
				//motor[motorI] = motor[motorI]);
			}
			delay(5);//delay 25ms
		}
	}
}//task for "slew"-ing the motres by adding to their power via loops
void baseMove(const struct sideBase* side, int speed) {
	int power = limitUpTo(127, speed);
	//motor[side->motors[0]] = power;//up is fast
	//motor[side->motors[1]] = power;//up is fast
	//motor[side->motors[2]] = power;//up is fast
	motorSlew[side->motors[0]] = power;
	motorSlew[side->motors[1]] = power;
	motorSlew[side->motors[2]] = power;
}
void driveLR(const int powerR, const int powerL) {
	if(autonRunning){//only do antistall during auton
		if(!Right.stalling) baseMove(&Right, powerR);
		else {
			baseMove(&Right, 0);
			delay(750);//stall waiter
		}
		if(!Left.stalling) baseMove(&Left, powerL);
		else {
			baseMove(&Left, 0);
			delay(750);//stall waiter
		}
	}
	else{
		baseMove(&Right, powerR);
		baseMove(&Left,  powerL);
	}
}
void driveCtrlr() {
	//scale for joystick
	const float partner = 1;//0.8;
	const float primary = 1;
	//driveLR(//truspeed taking both controllers
	//TruSpeed(limitUpTo(127, primary*vexRT[Ch2] + partner*vexRT[Ch2Xmtr2]), 3),
	//TruSpeed(limitUpTo(127, primary*vexRT[Ch3] + partner*vexRT[Ch3Xmtr2]), 3)
	//	);
	driveLR(//truspeed taking both controllers
	primary*vexRT[Ch2] + partner*vexRT[Ch2Xmtr2],
	primary*vexRT[Ch3] + partner*vexRT[Ch3Xmtr2]
	);
}
void fwds(const int power, const float angle = mRot) {//drive base forwards
	int speed = limitUpTo(127, power);
	const float scalar = 15;//scalar for rotation
	float dirSkew = limitUpTo(speed, scalar*(mRot - angle));
	driveLR(speed - dirSkew, speed + dirSkew);
}
void rot(const float speed) {//rotates base
	driveLR(speed, -speed);
}
void driveFor(float goal) {//drives for certain distance (arbitrary units)
	//works best from 1 to 40 ish.
	resetEncoders();
	//	goal *= 2;//doubles "goal" not tuned very well as of rn
	const int thresh = 5;//10 ticks
	int initDir = mRot;
	//ClearTimer(T1);
	float dP = 20;//multiplier for velocity controller
	float vel = velocity;
	const float encoderRatio = 0.5;
	while (abs(goal * circum - encoderAvg*encoderRatio) > thresh) {
		fwds(limitDownTo(15, dP * ((goal*circum - encoderAvg*encoderRatio - 0.1*vel))), initDir);
	}
	fwds(0);
	return;
}
void driveFor2(int goal) {//drives for certain distance in inches
	resetEncoders();
	const int thresh = 120;
	const int initDir = mRot;
	//const float encoderScale = 1;//number of motor rotations = this() rotations
	const float dP = 0.3;//25;//multiplier for velocity controller
	float goalTicks = goal*28.6479 ;
	while (abs(goalTicks - encoderAvg) > thresh) { //while error > threshold
		//encoder geared 1:1, circum = 4*pi
		//goal / 4pi = number of revolutions
		//360 ticks per revolution
		//therefore conversion to ticks is goal / 4pi * 360 => scalar of 28.6479
		fwds(limitDownTo(15, dP * (goalTicks - encoderAvg)));
	}
	fwds(sgn(goal) * -40, initDir);
	delay(30);
	fwds(0, initDir);
	//settle();
	return;
}
void rotFor(float target, float dP = 1.2){
	gyroBase.isRunning = true;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	while(abs(SensorValue[Gyro]*GyroK - target) > 0.5){
		rot(limitDownTo(20, dP*(target - SensorValue[Gyro]*GyroK) ) );
	}
	rot(-sgn(target)*60);
	delay(30);
	rot(0);
	return;
}
void RSwingFor(int target){
	gyroBase.isRunning = true;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	while(abs(SensorValue[Gyro]*GyroK - target) > 0.5){
		baseMove(&Right, limitDownTo(20, 4*(target - SensorValue[Gyro]*GyroK) ) );
	}
	baseMove(&Right,-sgn(target)*60);
	delay(30);
	rot(0);
	return;
}
void LSwingFor(int target){
	gyroBase.isRunning = true;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	while(abs(SensorValue[Gyro]*GyroK - target) > 0.5){
		if (target>0) baseMove(&Right, 25); //keep right side still moving back
			baseMove(&Left, limitDownTo(20, 3*(SensorValue[Gyro]*GyroK- target) ) );
	}
	baseMove(&Left,-sgn(target) *60);
	delay(30);
	rot(0);
	return;
}
#endif

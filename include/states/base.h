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
float TruSpeed(const float value, const int power) {//for all other polynomials; visit: goo.gl/mhvbx4
	int final;//only for powers of 2-5
	if (power == 2) final = getSign(value) * ( sqr(value) / 127 );				//squaring
	else if (power == 3) final = ( cube(value) / sqr(127) );					//cubing
	else if (power == 4) final = getSign(value) * ( quar(value) / cube(127) );	//4th degree
	else if (power == 5) final = ( cinq(value) / quar(127) );					//cubing
	else final = value;//nothing fanci
	return final;
}//function for calculating the truSpeed function based off a polynomial
void baseMove(const struct sideBase* side, int speed) {
	int power = limitUpTo(127, speed);
	motor[side->motors[0]] = power;//up is fast
	motor[side->motors[1]] = power;//up is fast
	motor[side->motors[2]] = power;//up is fast
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
	driveLR(//truspeed taking both controllers
	TruSpeed(limitUpTo(127, primary*vexRT[Ch2] + partner*vexRT[Ch2Xmtr2]), 3),
	TruSpeed(limitUpTo(127, primary*vexRT[Ch3] + partner*vexRT[Ch3Xmtr2]), 3)
	);
}
void fwds(const int power, const float angle = mRot) {//drive base forwards
	int speed = LIMITUP(127, power);
	const float scalar = 15;//scalar for rotation
	float dirSkew = LIMITUP(speed, scalar*(mRot - angle));
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
		fwds(LIMITDOWN(15, dP * ((goal*circum - encoderAvg*encoderRatio - 0.1*vel))), initDir);
	}
	//fwds(-GETSIGN(goal)* 80);
	//delay(50);
	fwds(0);
	return;
}
void driveFor2(int goal) {//drives for certain distance in inches
	//SensorValue[LeftBaseEnc] = 0;
	//SensorValue[RightBaseEnc] = 0;
	resetEncoders();
	const int thresh = 360;
	const int initDir = mRot;
	//const float encoderScale = 1;//number of motor rotations = this() rotations
	const float dP = 0.06;//25;//multiplier for velocity controller
	float goalTicks = goal*28.6479 ;
	while (abs(goalTicks - encoderAvg) > thresh) { //while error > threshold
		//encoder geared 1:1, circum = 4*pi
		//goal / 4pi = number of revolutions
		//360 ticks per revolution
		//therefore conversion to ticks is goal / 4pi * 360 => scalar of 28.6479
		fwds(limitDownTo(15, dP * abs(goalTicks - encoderAvg)));
		//fwds(LIMITDOWN(15, dP * ((goal*circum - encoderAvg*encoderRatio - 0.1*vel))), initDir);
	}
	fwds(GETSIGN(goal)*-127, initDir);
	delay(50);
	fwds(0, initDir);
	//settle();
	return;
}
void rotFor(float target, float dP = 1.2){
	gyroBase.isRunning = true;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	while(abs(SensorValue[Gyro]*GyroK - target) > 0.5){
		rot(LIMITDOWN(20, dP*(target - SensorValue[Gyro]*GyroK) ) );
	}
	rot(-GETSIGN(target)*60);
	delay(30);
	rot(0);
	return;
}
void RSwingFor(int target){
	gyroBase.isRunning = true;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	while(abs(SensorValue[Gyro]*GyroK - target) > 0.5){
		baseMove(&Right, LIMITDOWN(20, 3*(target - SensorValue[Gyro]*GyroK) ) );
	}
	baseMove(&Right,-GETSIGN(target)*60);
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
		baseMove(&Left, LIMITDOWN(20, 3*(SensorValue[Gyro]*GyroK- target) ) );
	}
	baseMove(&Left,-GETSIGN(target) *60);
	delay(30);
	rot(0);
	return;
}
#endif

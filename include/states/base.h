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
	int final;//only for powers of 2-5
	if (power == 2) final = getSign(value) * ( sqr(value) / 127 );				//squaring
	else if (power == 3) final = ( cube(value) / sqr(127) );					//cubing
	else if (power == 4) final = getSign(value) * ( quar(value) / cube(127) );	//4th degree
	else if (power == 5) final = ( cinq(value) / quar(127) );					//cubing
	else final = power;//nothing fanci
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
void fwds(const int power, const float angle) {//drive base forwards
	int speed = limitUpTo(127, power);
	const float scalar = 15;//scalar for rotation
	float dirSkew = limitUpTo(speed, scalar*(mRot - angle));
	driveLR(speed - dirSkew, speed + dirSkew);
}
void rot(const float speed) {//rotates base
	driveLR(speed, -speed);
}
void driveFor(float goal) {//drives for certain inches
	//works best from 1 to 40 ish.
	resetEncoders();
	goal *= 2;//doubles "goal" not tuned very well as of rn
	const int thresh = 5;//10 ticks
	int initDir = mRot;
	//ClearTimer(T1);
	float dP = 20;//multiplier for velocity controller
	float vel = velocity;
	const float encoderRatio = 0.5;
	while (abs(goal * circum - encoderAvg*encoderRatio) > thresh) {
		fwds(limitDownTo(15, dP * ((goal*circum - encoderAvg*encoderRatio - 0.1*vel))), initDir);
	}
	fwds(0, initDir);
	return;
}
void rotFor(float target){
	gyroBase.isRunning = true;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	while(abs(SensorValue[Gyro]*GyroK - target) > 0.5){//2 dF
		rot(limitDownTo(15, pidCompute(&gyroBase, (target)/GyroK)));
	}
	rot(0);//gives settle time
	gyroBase.isRunning = false;
	resetPIDVals(&gyroBase);
	delay(target * 2);
	//check for overshoots
	const int slowPower = 60;
		while(SensorValue[Gyro]*GyroK > target + 0.5)
			rot(-abs(slowPower));
		while(SensorValue[Gyro]*GyroK < target - 0.5)
			rot(+abs(slowPower));
	rot(0);
	return;
}
#endif

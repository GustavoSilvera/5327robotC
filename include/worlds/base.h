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
		if(slewRating){
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
		else return;
	}
}//task for "slew"-ing the motres by adding to their power via loops
void baseMove(const struct sideBase* side, int speed) {
	int power = limitUpTo(127, speed);
	if(!slewRating){
		motor[side->motors[0]] = power;//up is fast
		motor[side->motors[1]] = power;//up is fast
		motor[side->motors[2]] = power;//up is fast
	}
	else{
		motorSlew[side->motors[0]] = power;
		motorSlew[side->motors[1]] = power;
		motorSlew[side->motors[2]] = power;
	}
}
void driveLR(const int powerR, const int powerL) {
	if(autonRunning){//only do antistall during auton
		if(!Right.isStalling) baseMove(&Right, powerR);
		else {
			baseMove(&Right, 0);
			delay(500);//stall waiter
		}
		if(!Left.isStalling) baseMove(&Left, powerL);
		else {
			baseMove(&Left, 0);
			delay(500);//stall waiter
		}
	}
	else{
		baseMove(&Right, powerR);
		baseMove(&Left,  powerL);
	}
	motor[Base_B] = avg2(powerR, powerL);
}
void driveCtrlr() {
	//scale for joystick
	int rotAxis = vexRT[Ch4Xmtr2];
	/*driveLR(//truspeed taking both controllers
		TruSpeed(limitUpTo(127, primary*vexRT[Ch2]),1) + rotAxis,// + partner*vexRT[Ch2Xmtr2]), 3),
		TruSpeed(limitUpTo(127, primary*vexRT[Ch3]),1) - rotAxis// + partner*vexRT[Ch3Xmtr2]), 3)
	);*/
	driveLR(//NO truspeed taking both controllers
		vexRT[Ch2] + rotAxis,
		vexRT[Ch3] - rotAxis
	);
	if(sgn(vexRT[Ch2]) != sgn(vexRT[Ch3])) motor[Base_B] = 0; //don't use when turning
	else if(abs(vexRT[Ch2]) > 15 && abs(vexRT[Ch3]) > 15 ) motor[Base_B] = avg2(vexRT[Ch2], vexRT[Ch3]); //if same direction
	if(abs(vexRT[Ch2]) < 10 && abs(vexRT[Ch3]) > 50 ) motor[Base_B] = vexRT[Ch3];//swing turning left
	if(abs(vexRT[Ch3]) < 10 && abs(vexRT[Ch2]) > 50 ) motor[Base_B] = vexRT[Ch2];//swing turning left

}
void fwds(const int power, const float angle = SensorValue[Gyro]*GyroK) {//drive base forwards
	int speed = limitUpTo(127, power);
	const float scalar = 15;//scalar for rotation
	float dirSkew = limitUpTo(speed, scalar*(angle - angle));
	driveLR(speed - dirSkew, speed + dirSkew);
}
void rot(const float speed) {//rotates base
	driveLR(speed, -speed);
}
void driveFor(int goal, int dir = SensorValue[Gyro]*GyroK) {//drives for certain distance in inches
	SensorValue[BaseEncoder] = 0;
	//delay(1000);//for debugging purposes
	const int thresh = 60;
	const float dP = 0.183;//25;//multiplier for velocity controller
	float goalTicks = goal*86.4211342;
	while (abs(goalTicks - SensorValue[BaseEncoder]) > thresh) { //while error > threshold
		//encoder geared 3:1, circum = 4*pi
		//goal / 4pi = number of revolutions
		//360 ticks per revolution
		//gear ratio of 1:3, so multiply by 3
		//therefore conversion to ticks is goal / 4pi * 360 * 3 => scalar of 86.4211342
		fwds(limitDownTo(15, dP * (goalTicks - encoderAvg)), dir);
	}
	fwds(sgn(goal) * -20, dir);
	delay(100);
	fwds(0, dir);
	//settle();
	return;
}
void rotFor(float target, float dP = 2){
	int power = 0;
	gyroBase.isRunning = true;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	clearTimer(T4);
	while(abs(SensorValue[Gyro]*GyroK - target) > 1 && time1[T4] < abs(target)*20){
		//if(time1[T4] > 1000) push += 1; //will gradually increase if bot does not turn in time
		power = limitDownTo(22, dP*(target - SensorValue[Gyro]*GyroK)); //+ push ;
		rot(power);
	}
	rot(-sgn(target)*60);
	delay(30);
	rot(0);
	return;
}
/*void curveFor(int dir, int target, float prop = 0.5){
	int power = 0;
	gyroBase.isRunning =true;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	clearTimer(T4);
	while(abs(SensorValue[Gyro]*GyroK - target) > 1 && time1[T4] < abs(target)*40){
		power = limitDownTo(22, 0.05*(target - SensorValue[Gyro]*GyroK));
		if(target > 0){ //anti-clockwise
			baseMove(&Right, dir * power);
			baseMove(&Left, -dir * power * prop);
		}
		else{ //clockwise
			baseMove(&Left, dir * power);
			baseMove(&Right, -dir * power * prop);
		}
		motor[Base_B] = dir * power * prop;//back motor moves with dir regardless of target angle
	}
	baseMove(&Left, 0);
	baseMove(&Right, 0);
	motor[Base_B] = 0;
	return;
}*/
void RSwingFor(int target){
	gyroBase.isRunning = true;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	while(abs(SensorValue[Gyro]*GyroK - target) > 4){
		if (target>0) baseMove(&Left, -30); //keep right side still moving back
		baseMove(&Right, limitDownTo(40, 7.5*(target - SensorValue[Gyro]*GyroK) ) );
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
		baseMove(&Left, limitDownTo(40, 7.5*(SensorValue[Gyro]*GyroK- target) ) );
	}
	baseMove(&Left,-sgn(target) *60);
	delay(30);
	rot(0);
	return;
}
task LockNLoad(){
	//int initLock = SensorValue[SonarR];
	const int sonarPos = 2030;
	clearTimer(T3);
	while(time1[T3] < 500){
		fwds(0.1*(SensorValue[SonarR] - sonarPos));
		if(D8) break;
		delay(10);
	}
	const int initialBase = SensorValue[BaseEncoder];
	for (;;) {//base PID lock
		fwds(0.2*(SensorValue[BaseEncoder] - initialBase));
		if (D8) break;
		delay(10);
	}
	return;
}
#endif

//#include "include/skills/other.h"
//function for driving the robot
void swingL(int power){
	motor[LBaseFront] = -power;
	motor[LBaseBack] = -power;
}
void swingR(int power){
	motor[RBaseFront] = power;
	motor[RBaseBack] = power;
}
void fwds(const int power, const float angle = mRot) {//drive base forwards
	const int speed = LimitUpTo(127, power);
	float dirSkew = LimitUpTo(speed, 15 * (mRot - angle) );
	motor[RBaseFront] = speed - dirSkew;
	motor[RBaseBack] = speed - dirSkew;
	motor[LBaseFront] = speed + dirSkew;
	motor[LBaseBack] = speed + dirSkew;
}
bool nearStopped(int velThresh, int motorThresh){
	return(abs(mainVelocity) < velThresh && //low base velocity
	abs(motor[baseRight.m.motors[0]]) < motorThresh && //low motor powers
	abs(motor[baseLeft.m.motors[0]])  < motorThresh &&
	abs(motor[baseRight.m.motors[1]]) < motorThresh &&
	abs(motor[baseLeft.m.motors[1]])  < motorThresh
	);
}
void settle(){
	while(!nearStopped(20, 20))//threshold for waiting
		fwds(0);
	return;
}
void driveFor(const int goal, bool angularCorrectionEnabled = false) {//drives for certain inches
	SensorValue[LeftBaseEnc] = 0;
	SensorValue[RightBaseEnc] = 0;
	const int thresh = 360;
	const float initDir = mRot;
	//const float encoderScale = 1;//number of motor rotations = this() rotations
	const float dP = 0.2;//0.06;//25;//multiplier for velocity controller
	goalTicks = goal*114.5916 ;
	while (abs(goalTicks - encoderAvg) > thresh) { //while error > threshold
		//encoder geared down 4:1, circum = 4*pi
		//goal / 4pi = number of revolutions
		//360 ticks per revolution
		//therefore conversion to ticks is goal / 4pi * 360 * 4 => scalar of 114.5916
		goalPower = getSign(goal) * LimitDownTo(15, dP * abs(goalTicks - encoderAvg));
		if(angularCorrectionEnabled) fwds(goalPower, initDir);
		else fwds(goalPower);
	}
	fwds(0);
	return;
}
void alignToLine(float dir){
	const float lineThresh = 1500;
	bool isAligned = (SensorValue[RLin] + SensorValue[LLin] < lineThresh);
	const int power = dir * 35;
	clearTimer(T2);
	while(!isAligned && (time1(T2) < 2000)){
		if(SensorValue[LLin] < lineThresh/2){
			fwds(0, mRot);
			while(SensorValue[RLin] > lineThresh/2){
				swingR(1.5*power);
			}
			isAligned = true;
		}
		else if (SensorValue[RLin] < lineThresh/2){
			fwds(0, mRot);
			while(SensorValue[LLin] > lineThresh/2){
				swingL(-1.5*power);
			}
			isAligned = true;
		}
		else fwds(power);// fwds(power, mRot); //speed where robot will stop on line
	}
	fwds(0);
}
void alignToDark(float dir){
	const float lineThresh = 1500;
	bool isAligned = (SensorValue[RLin] + SensorValue[LLin] > lineThresh);
	const int power = dir * 75;
	while(!isAligned){
		if(SensorValue[LLin] > lineThresh){
			fwds(0, mRot);
			while(SensorValue[RLin] < lineThresh){
				swingR(power);
			}
			isAligned = true;
		}
		else if (SensorValue[RLin] > lineThresh){
			fwds(0, mRot);
			while(SensorValue[LLin]< lineThresh){
				swingL(power);
			}
			isAligned = true;
		}
		else fwds(power*0.5);// fwds(power, mRot); //speed where robot will stop on line
	}
	fwds(0);
}
void driveToSonar(bool side, int goal, float kP=6){ //use sonar to reach distance in inches
	if(side == RIGHT) while(SensorValue[Rsonar] > goal) fwds(kP*(SensorValue[Rsonar] - goal));
	else while(SensorValue[Lsonar] > goal) fwds(kP*(SensorValue[Lsonar] - goal));
	return;
}

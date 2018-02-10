//function for driving the robot

void swingR(float power) {//swings right base
	analogMechControl(&baseRight.m, power);
}
void swingL(float power) {//swings right base
	analogMechControl(&baseRight.m, power);
}
void fwdsLong(const int power, const float angle = mRot) {//drive base forwards
	const float speed = LimitUpTo(127, power);
	//const float scalar = 0;//scalar for rotation (CHANGED)
	//float dirSkew = LimitUpTo(speed, scalar*(mRot - angle));
	motor[RBaseFront] = speed;
	motor[RBaseBack] = speed;
	motor[LBaseFront] = speed*0.625;
	motor[LBaseBack] = speed*0.625;
	//analogMechControl(&baseLeft.m, speed + dirSkew);
	//analogMechControl(&baseRight.m, speed - dirSkew);
}

void fwds(const int power, const float angle = mRot) {//drive base forwards
	const float speed = LimitUpTo(127, power);
	motor[RBaseFront] = speed;
	motor[RBaseBack] = speed;
	motor[LBaseFront] = speed;
	motor[LBaseBack] = speed;
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
	while(!nearStopped(30, 20)){//threshold for waiting
		fwds(0, mRot);
	}
	return;
}
void driveFor(float goal) {//drives for certain inches
	SensorValue[LeftBaseEnc] = 0;
	SensorValue[RightBaseEnc] = 0;
	const int thresh = 5;//10 ticks
	const int initDir = mRot;
	//ClearTimer(T1);
	//const float encoderScale = 1;//number of motor rotations = this() rotations
	const float dP = 20;//25;//multiplier for velocity controller
	//while (abs(goal * circum - encoderAvg) > thresh) {
	if(goal < 40){
		while (abs(goal * circum - SensorValue[LeftBaseEnc]*0.75) > thresh) {
			fwds(LimitDownTo(15, dP * ((goal*circum - SensorValue[LeftBaseEnc]*0.75 - 0.3*mainVelocity))), mRot);//initDir);
		}
	}
	else{
		while (abs(goal * circum - SensorValue[LeftBaseEnc]*0.75) > thresh) {
			fwdsLong(LimitDownTo(15, dP * ((goal*circum - SensorValue[LeftBaseEnc]*0.75 - 0.3*mainVelocity))), mRot);//initDir);
		}
	}

	fwds(0, initDir);
	settle();
	return;
}
void alignToLine(float dir){
	const float lineThresh = 1000;
	bool isAligned = (SensorValue[RLin] + SensorValue[LLin] < lineThresh);
	const int power = dir * 65;
	while(!isAligned){
		if(SensorValue[LLin] < lineThresh){
			fwds(0, mRot);
			while(SensorValue[RLin] > lineThresh){
				swingR(power);
			}
			isAligned = true;
		}
		else if (SensorValue[RLin] < lineThresh){
			fwds(0, mRot);
			while(SensorValue[LLin] > lineThresh){
				swingL(power);
			}
			isAligned = true;
		}
		else fwds(dir*65, mRot);// fwds(power, mRot); //speed where robot will stop on line
	}
	fwds(0, mRot);
}
void untilLine(int power){
	const float lineThresh = 1000;
	while((SensorValue[LLin] > lineThresh) && (SensorValue[RLin] > lineThresh)){
		fwds(power, mRot);
	}
	fwds(0, mRot);
}
void alignSonar(int goal){ //use sonar to reach distance in inches
	while(SensorValue[sonar] > goal){
		fwds(127, mRot);
	}
	fwds(-127, mRot);
	delay(50);
}

#if !defined(SENSOR_H)
#define SENSOR_H
/*
 .----------------.  .----------------.  .-----------------. .----------------.  .----------------.  .----------------.
| .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
| |    _______   | || |  _________   | || | ____  _____  | || |    _______   | || |     ____     | || |  _______     | |
| |   /  ___  |  | || | |_   ___  |  | || ||_   \|_   _| | || |   /  ___  |  | || |   .'    `.   | || | |_   __ \    | |
| |  |  (__ \_|  | || |   | |_  \_|  | || |  |   \ | |   | || |  |  (__ \_|  | || |  /  .--.  \  | || |   | |__) |   | |
| |   '.___`-.   | || |   |  _|  _   | || |  | |\ \| |   | || |   '.___`-.   | || |  | |    | |  | || |   |  __ /    | |
| |  |`\____) |  | || |  _| |___/ |  | || | _| |_\   |_  | || |  |`\____) |  | || |  \  `--'  /  | || |  _| |  \ \_  | |
| |  |_______.'  | || | |_________|  | || ||_____|\____| | || |  |_______.'  | || |   `.____.'   | || | |____| |___| | |
| |              | || |              | || |              | || |              | || |              | || |              | |
| '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
 '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------'
*/
void scaleGyros(){
	SensorScale[Gyro] = 260;
	//SensorBias[Gyro] = 1;
	SensorFullCount[Gyro] = 3600;
}
void gyroBias(){
	int realBias;
    long cumulativeBias;
    // Cause the gyro to reinitialize (a theory anyway)
    SensorType[Gyro] = sensorNone;
    // Wait 1/2 sec
    wait10Msec(50);
    // improved bias setting
    cumulativeBias=0;
    //calculate avereage bias over 1 second
    const int durationTime = 1000;
    for(int i=0;i<durationTime;i++){
	    cumulativeBias += SensorValue[Gyro];
	    wait1Msec(1);
    }
    realBias = cumulativeBias/durationTime;
    // Gyro should be motionless here
    SensorType[Gyro] = sensorGyro;
    // Now put the bias back to the correct value we found in the initialization
    SensorBias[Gyro] = realBias;
}
void resetGyros() {
	//SensorType[in1] = sensorNone;
	//SensorType[in1] = sensorGyro;//resets gyro sensor, rly sketchy
	SensorValue[Gyro] = 0;//resets gyro sensor
	delay(300);
	scaleGyros();
}
void resetEncoders(){
	SensorValue[LeftEncoder] = 0;
	SensorValue[RightEncoder] = 0;
}
/*ANTI-STALLING STUFF*/
bool doubleTap(){
	if(vexRT[Btn8L] == 1){
		clearTimer(T3);
		delay(100);
		while(time1[T3] < 400){
			if(vexRT[Btn8L] == 1) return true;
		}
		return false;
	}
	return false;
}
bool stalling(const struct sideBase* side){
	return (
	abs(motor[side->motors[0]]) >= 80 &&//high ish power
	//abs(motor[side->motors[0]]) > 80 &&//high ish power
	abs(side->velocity) < 20//low ish velocity yet high speed (for like 500 ms)
	);
}
void checkStalling(struct sideBase* side){
	if(stalling(side)){
		clearTimer(T1);
		bool currentlyStalling = true;
		while(time1[T1] < 150){//checkingn for continuous stalling (else instantanious refresh)
			currentlyStalling = stalling(side);//still stalling
			if(currentlyStalling) continue;//keep going until time limit
			else break;
		}
		if(currentlyStalling){//done waiting, final check
			playSound(soundBlip);
			side->stalling = true;//if so, consider it stalling
		}
	}
	else side->stalling = false;
	//if(side->stalling) playSound(soundShortBlip);
}
task antiStall(){
	for(;;){
		//return;
		//checkStalling(&Right);
		//checkStalling(&Left);
		if(!autoStacking) checkStalling(&goliat);
		delay(20);
	}
}
/********************************************************************************\
* .----------------.  .----------------.  .----------------.  .----------------. *
*| .--------------. || .--------------. || .--------------. || .--------------. |*
*| |   ______     | || |  ____  ____  | || |  ____  ____  | || |    _______   | |*
*| |  |_   __ \   | || | |_   ||   _| | || | |_  _||_  _| | || |   /  ___  |  | |*
*| |    | |__) |  | || |   | |__| |   | || |   \ \  / /   | || |  |  (__ \_|  | |*
*| |    |  ___/   | || |   |  __  |   | || |    \ \/ /    | || |   '.___`-.   | |*
*| |   _| |_      | || |  _| |  | |_  | || |    _|  |_    | || |  |`\____) |  | |*
*| |  |_____|     | || | |____||____| | || |   |______|   | || |  |_______.'  | |*
*| |              | || |              | || |              | || |              | |*
*| '--------------' || '--------------' || '--------------' || '--------------' |*
* '----------------'  '----------------'  '----------------'  '----------------' *
\********************************************************************************/
float calcVel(const struct liftMech* lift, const float dist, const float delayAmount) {
	float velocity = ((SensorValue(lift->sensor) - lift->past) / dist) / ((float)(delayAmount / 1000));//1000ms in 1s
	lift->past = SensorValue[lift->sensor];
	return (velocity);//1000 ms in 1s;
}
float calcVel(const struct sideBase* side, const float dist, const float delayAmount) {
	float velocity = limitDownTo(1, ((SensorValue(side->sensor) - side->past) / dist) / ((float)(delayAmount / 1000)));//1000 ms in 1s;
	side->past = SensorValue[side->sensor];
	return(velocity);//1000 ms in 1s;
}
float calcRotVel(){
	float velocity = SensorValue[Gyro] - pastRot;
	pastRot = SensorValue[Gyro];
	return(velocity);
}
task MeasureSpeed() {
	/*MEASURING IN IN/SEC*/
	const float dist = 1.125*PI;
	const float delayAmount = 50;
	for (;;) {
		mRot = (float)(GyroK*SensorValue[Gyro]);
		encoderAvg = SensorValue[Right.sensor];//avg2(SensorValue[Right.sensor], SensorValue[Left.sensor]);
		//base velocity
		Right.velocity = calcVel(&Right, circum, delayAmount);
		Left.velocity = calcVel(&Left, circum, delayAmount);
		velocity = avg2(Right.velocity, Left.velocity);//overall velocity (avdg between the two)
		rotVelocity = calcRotVel();//calculates rotational velocity
		//lift velocities
		mainLift.velocity = calcVel(&mainLift, dist, delayAmount);
		FourBar.velocity = calcVel(&FourBar, dist, delayAmount);
		goliat.velocity = calcVel(&goliat, dist, delayAmount);
		//does the waitings
		delay(delayAmount);
	}
}//task for measuring velocity of the base, in IN/Sec

#endif

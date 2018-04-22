#if !defined(LIFT_H)
#define LIFT_H
/*******************************************************************************\
* .----------------.  .----------------.  .----------------.  .----------------. *
*| .--------------. || .--------------. || .--------------. || .--------------. |*
*| |   _____      | || |     _____    | || |  _________   | || |  _________   | |*
*| |  |_   _|     | || |    |_   _|   | || | |_   ___  |  | || | |  _   _  |  | |*
*| |    | |       | || |      | |     | || |   | |_  \_|  | || | |_/ | | \_|  | |*
*| |    | |   _   | || |      | |     | || |   |  _|      | || |     | |      | |*
*| |   _| |__/ |  | || |     _| |_    | || |  _| |_       | || |    _| |_     | |*
*| |  |________|  | || |    |_____|   | || | |_____|      | || |   |_____|    | |*
*| |              | || |              | || |              | || |              | |*
*| '--------------' || '--------------' || '--------------' || '--------------' |*
* '----------------'  '----------------'  '----------------'  '----------------' *
\*******************************************************************************/
void liftMove(const struct liftMech* lift, const float speed) {
	float power = limitUpTo(127, speed);
	motor[lift->motors[0]] = power;//full speed
	motor[lift->motors[1]] = power;//full speed
}
void liftDiff(const struct liftMech* lift, const float speed) {
	float power = limitUpTo(127, speed);
	motor[lift->motors[0]] = power;//full speed
	motor[lift->motors[1]] = -power;//isReversed for differential
}
void resetPIDVals(const struct PIDs* pid) {
	pid->Last = 0;
	pid->Integral = 0;
	pid->Derivative = 0;
}
float pidCompute(const struct PIDs* PID, float current) {
	float error = current - PID->goal;//calculate error
	//if (abs(error) < PID->thresh);
	int dir = 1;
	int power = 0;
	if (PID->isReversed) dir = -1;
	const float untilIntegral = PID->thresh;//considered "low threshold" for potentiometers
	// calculate the integral
	if (PID->kI != 0.0) {//calculates integral (only at very end)
		if (abs(error) < untilIntegral) PID->Integral += error;//used for averaging the integral amount, later in motor power divided by 25
		else PID->Integral = 0.0;
		power += PID->kI * PID->Integral;
	}
	else PID->Integral = 0.0;
	// calculate the derivative
	if (PID->kD != 0.0) {
		PID->Derivative = error - PID->Last;//change in errors
		PID->Last = error;
	}
	else PID->Derivative = 0.0;
	if(abs(error) > PID->thresh) {
		power += PID->kP * error;
		power += PID->kD * PID->Derivative;
	}
	if(abs(power) < 25) power = 0; //lowest power = 15
	return dir * power;
}
void enablePID(struct liftMech* lift){
	lift->PID.goal = SensorValue[lift->sensor];//keeps lift in last position
	lift->PID.isRunning = true;//re-enables pid
}
void PIDLift(const struct liftMech* lift) {
	if (lift->PID.isRunning)
		liftMove(lift, pidCompute(lift->PID, SensorValue[lift->sensor]));//power the lift with its PID
	else resetPIDVals(lift->PID);//turn off the PID and reset values
	delay(lift->PID.refresh);//delay a lil bit
}
void UpUntil(const struct liftMech* lift, int goal, int speed = 127) {
	lift->PID.isRunning = false;
	while (SensorValue[lift->sensor] < goal){ //brings lift up to goal
		if(lift->type == DIFFERENTIAL) liftDiff(lift, -speed);
		else liftMove(lift, speed);
	}
	liftMove(lift, -abs(speed));
	delay(10);
	enablePID(lift);
	return;
}
void liftMoveT(struct liftMech* lift, int power, int delayTime){//time based lift move
	clearTimer(T4);
	while(time1[T4] < delayTime) liftMove(lift, power);
	liftMove(lift, 0);
	return;
}
void UpUntilW4Bar(int goal, float prop, int speed, bool FourBarToMax) {
	//mainLift.PID.isRunning = false;
	mainLift.PID.isRunning = true;
	int offset = 0;
	if(currentCone >= 13) offset = 100;
	playSound(soundBeepBeep);
	while (SensorValue[mainLift.sensor] < goal){ //brings lift up to goal
		playSound(soundShortBlip);
		liftMove(mainLift, abs(speed));
		if(SensorValue[mainLift.sensor] > goal * prop){
			if(FourBarToMax) FourBar.PID.goal = FourBar.max - offset;
			else FourBar.PID.goal = FourBar.min;
			FourBar.PID.isRunning = true;
		}
		else liftDiff(&goliat, 127);//if not moving 4bar, hold the cone more
	}
	liftMove(mainLift, 0);
	//mainLift.PID.isRunning = true;
	return;
}
void DownUntil(struct liftMech* lift, int goal, int speed = 127) {
	lift->PID.isRunning = false;
	while (SensorValue[lift->sensor] > goal ){//brings lift down to goal
		if(lift->type == DIFFERENTIAL) liftDiff(lift, speed);
		else liftMove(lift, -abs(speed));
	}
	enablePID(lift);
	return;
}
void manualLiftControl(const struct liftMech* lift, int up1, int up2, int dwn1, int dwn2, int maxSpeed = 127) {
	int dir = 1;
	int power = 0;
	lift->PID.isRunning = false;
	if (lift->isReversed) dir = -1;
	bool upButton 		=	(up1 || up2 );//defining what is up button
	bool downButton 	=	(dwn1 || dwn2 );//defining what is down button
	bool withinUpper 	=	(SensorValue[lift->sensor] <= lift->max || lift->type == DIFFERENTIAL || lift->type == NOSENSOR);//within upper bound
	bool withinLower 	=	(SensorValue[lift->sensor] >= lift->min || lift->type == DIFFERENTIAL || lift->type == NOSENSOR);//within lower bound
	if(upButton && withinUpper) power = up1*maxSpeed + up2*0.5*maxSpeed;
	else if (downButton && withinLower) power = -(dwn1*maxSpeed + dwn2*0.5*maxSpeed);
	else power = 0;
	if(lift->type == DIFFERENTIAL){
		if(power != 0) {
			mainLift.PID.isRunning = false;
			FourBar.PID.isRunning = false;
			liftDiff(lift, dir * power);//main lift and 4bar are normal-er
		}
		else return;
	}
	else if(lift->type == BINARY){
		lift->PID.isRunning = true;
		if(power > 0) {
			//change PID value
			lift->PID.kP =0.10;
			liftMove(lift, 127);
			lift->PID.goal = lift->max;//holdTo(lift, true);//go to max
		}
		else if (power < 0) {
			//change PID value
			lift-> PID.kP = 0.20;
			liftMove(lift, -127);
			lift->PID.goal = lift->min;//holdTo(lift, false);//go to min
		}
		else lift->PID.goal = SensorValue[lift->sensor];
		liftMove(lift, dir * power);
	}
	else liftMove(lift, dir * power);//MoGo has motors going opposite speeds
	return;
}
void LiftLift(const struct liftMech* lift, int up1, int up2, int dwn1, int dwn2, float velLimit = 100) {
	if (up1 || up2 || dwn1 || dwn2){
		manualLiftControl(lift, up1, up2, dwn1, dwn2);
	}
	else if(lift->type != NOPID && lift->type != DIFFERENTIAL && lift->type != HOLD){//INTAKE & DIFFERENTIAL is only type without PID
		if (abs(lift->velocity) < velLimit) {
			if (!lift->PID.isRunning) lift->PID.goal = SensorValue[lift->sensor];//sets goal if not already running
			lift->PID.isRunning = true;//now pid is definitely running
		}
		else {
			lift->PID.isRunning = false;
			liftMove(lift, 0);//decelerate by turning off motors
		}
		if( abs(SensorValue[lift->sensor] - lift->PID.goal) < 80) lift->PID.kP = 0.2;
		PIDLift(lift);//calls the pid function for the lifts
	}
	else if (lift->type == NOPID) liftMove(lift, 0);
	else if (lift->type == HOLD) liftMove(lift, holdPower);
	else if (lift->type == DIFFERENTIAL) {
		lift->PID.isRunning = false;
		return;//dont even touch motors
	}
	else return;
}

void mainLiftLift(int up1, int up2, int dwn1, int dwn2){
	if((up1 || up2) && SensorValue[LiftPot] < mainLift.max){
		liftMove(&mainLift, 127);
	}
	else if((dwn1 || dwn2) && SensorValue[LiftPot] > mainLift.min){
		liftMove(&mainLift, -127);
	}
	else{ //with joystick control
		liftMove(&mainLift, TruSpeed(vexRT[Ch2Xmtr2],3));
		//liftMove(&mainLift, 0);
	}
}
task manualGoliath(){
	for(;;){
		if(!autonRunning){
			bool in = (L8 || U6_2);
			bool out= (R8 || D6_2);
			if(autoStacking){
				liftMove(&goliat, intakeSpeed);
				hasCone = false;
			}
			else if(in){
				//if told, full power
				liftMove(&goliat, 127);
			}
			else if(out){
				liftMove(&goliat, -127);
				hasCone = false;
				delay(500);
			}
			else if(goliat.velocity < 30){
				//if cone intaken, reduce intake power
				if(!hasCone) {
					playSound(soundFastUpwardTones);//detection noise
					clearTimer(T4);
					while(time1[T4] < 200){
						liftMove(&goliat, 127);
					}
				}
				liftMove(&goliat, 30);
				hasCone = true;
				flash();
			}
			else {
				hasCone = false;//timer will keep going for .5sec after button release
				liftMove(&goliat, 127);
			}
		}
		delay(5);
	}
}
bool notButtons(int u1, int u2, int d1, int d2){
	if(!u1 && !u2 && !d1 && !d2)	return true;
	else return false;
}
task LiftControlTask() {
	#define VBarBtns U5, U5_2, D5, D5_2
	#define LiftBtns U6, false, D6, false
	#define MoGoBtns U8, U8_2, D8, D8_2
	for (;;) {//while true
		if(!autonRunning){
			if(notButtons(MoGoBtns)) 	mainLiftLift(LiftBtns);//LiftLift(&mainLift, LiftBtns, 300);
			else LiftLift(&MoGo, MoGoBtns);
			LiftLift(&FourBar,  VBarBtns, 5000);
		}
		else {
			PIDLift(&mainLift);//calls the pid function for the lifts
			PIDLift(&FourBar);//calls the pid function for the lifts
		}
		delay(10);
	}
}
#endif

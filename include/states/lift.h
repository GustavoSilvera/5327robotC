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
	//float power = limitUpTo(127, speed);
	motor[lift->motors[0]] = speed;//full speed
	motor[lift->motors[1]] = speed;//full speed
}
void liftDiff(const struct liftMech* lift, const float speed) {
	float power = limitUpTo(127, speed);
	motor[lift->motors[0]] = power;//full speed
	motor[lift->motors[1]] = -power;//reversed for differential
}
void resetPIDVals(const struct PIDs* pid) {
	pid->LastError = 0;
	pid->Integral = 0;
	pid->Derivative = 0;
}
float pidCompute(const struct PIDs* PIDtype, const int goal) {
	float error = SensorValue[PIDtype->sensor] - goal;//calculate error
	if (abs(error) < PIDtype->thresh) error = 0;
	float untilIntegral = 100;//considered "low threshold" for potentiometers
	if (PIDtype->kI != 0) {//calculates integral (only at very end)
		if (abs(error) < untilIntegral) PIDtype->Integral += error;//used for averaging the integral amount, later in motor power divided by 25
		else PIDtype->Integral = 0;
	}
	else PIDtype->Integral = 0;
	// calculate the derivative
	PIDtype->Derivative = error - PIDtype->LastError;//change in errors
	PIDtype->LastError = error;
	// calculate drive (in this case, just for the lifts)
	int dir = 1;
	if (PIDtype->reversed) dir = -1;
	return dir * PIDtype->kP * error + PIDtype->kI * PIDtype->Integral + PIDtype->kD * PIDtype->Derivative;
	//return(dir * getSign(error) * abs((PIDtype->kP * error) + (PIDtype->kI * PIDtype->Integral) + (PIDtype->kD * PIDtype->Derivative)));
}
void enablePID(struct liftMech* lift){
	lift->goal = SensorValue[lift->sensor];//keeps lift in last position
	lift->PID.isRunning = true;//re-enables pid
}
void PIDLift(const struct liftMech* lift) {
	if (lift->PID.isRunning) liftMove(lift, pidCompute(lift->PID, lift->goal));//power the lift with its PID
	else resetPIDVals(lift->PID);//turn off the PID and reset values
	delay(lift->liftPIDelay);//delay a lil bit
}
void UpUntil(const struct liftMech* lift, int goal, int speed = 127) {
	lift->PID.isRunning = false;
	while (SensorValue[lift->sensor] < goal) //brings lift up to goal
		liftMove(lift, abs(speed));
	liftMove(lift, -abs(speed));
	delay(10);
	enablePID(lift);
	return;
}
void UpUntilW4Bar(int goal, float prop, int speed, bool FourBarToMax) {
	mainLift.PID.isRunning = false;
	while (SensorValue[mainLift.sensor] < goal){ //brings lift up to goal
		liftMove(mainLift, abs(speed));
		if(SensorValue[mainLift.sensor] > goal * prop){
			if(FourBarToMax) FourBar.goal = FourBar.max;
			else FourBar.goal = FourBar.min;
			FourBar.PID.isRunning = true;
		}
	}
	if(FourBarToMax)liftMove(mainLift, -abs(speed*0.5));
	delay(10);
	return;
}
int first = 0;
int last = 0;
int lastValue = 0;
/*void UpUntilSonar(const struct liftMech* lift, int speed = 127) {
	last = first;
	lift->PID.isRunning = false;
	const int threshold = 20;//10cm from cone max
	while (SensorValue[LiftPot] > 2400 && SensorValue[ultraSound] < threshold &&SensorValue[ultraSound] != -1 &&SensorValue[LiftPot] > lastValue) //brings lift up to goal
		liftMove(lift, abs(speed));
	lastValue = SensorValue[LiftPot];
	first = SensorValue[mainLift.sensor];
	playSound(soundBlip);
	liftMove(lift, -abs(speed));//quick hard stop
	delay(10);
	enablePID(lift);
	return;
}*/
void soundCompare(){
	if(abs(first - last) > 100) playSound(soundException);
	else playSound(soundUpwardTones);
}
void DownUntil(struct liftMech* lift, int goal, int speed = 127) {
	lift->PID.isRunning = false;
	while (SensorValue[lift->sensor] > goal )//brings lift down to goal
		liftMove(lift, -abs(speed));
	enablePID(lift);
	return;
}
void manualLiftControl(const struct liftMech* lift, int bUp, int bDown, int bUp2, int bDown2, bool reversed, int maxSpeed) {
	int dir = 1;
	int power;
	//int sensorVal = SensorValue[lift->sensor];
//	if(lift->type == DIFFERENTIAL) sensorVal = (4095 - SensorValue[lift->sensor]);
	if (reversed) 			(dir = -1);
	bool upButton 		=	(bUp == 1 || bUp2 == 1);//defining what is up button
	bool downButton 	=	(bDown == 1 || bDown2 == 1);//defining what is down button
	bool withinUpper 	=	(SensorValue[lift->sensor] <= lift->max);//within upper bound
	bool withinLower 	=	(SensorValue[lift->sensor] >= lift->min);//within lower bound
	if (!upButton && !downButton) power = 0;//not pressed any buttons
	else if ( (!withinUpper && upButton) || (!withinLower && downButton)) power = 0;//pressing buttons but !within bounds
	else if (upButton) 	 power =  dir * maxSpeed;//up max speed
	else if (downButton)  power = -dir * maxSpeed;//down max speed
	else 	power = 0;//anything else? just kill it
	if(lift->type == DIFFERENTIAL){
		if(abs(power) > 0) {
			mainLift.PID.isRunning = false;
			liftDiff(lift, power);//main lift and 4bar are normal-er
		}
		else return;
	}
	else if (lift->type == BINARY) {
		if(getSign(power) > 0) {
			UpUntil(lift, lift->max);
			lift->PID.kP = 0.5;
			lift->PID.thresh = 50;
			lift->goal = lift->max + 50;
		}
		else if(getSign(power) < 0){
			lift->PID.kP = 0.9;
			lift->PID.thresh = 30;
			DownUntil(lift, lift->min);
			lift->goal = lift->min - 200;
		}
		else liftMove(lift, 0);
	}
	else liftMove(lift, power);//mogo has motors going opposite speeds
}
void LiftLift(const struct liftMech* lift, int bUp, int bDown, int bUp2, int bDown2, bool reversed, float velLimit = 100) {
	if (bUp || bDown || bUp2 || bDown2) {
		lift->PID.isRunning = false;
		manualLiftControl(lift, bUp, bDown, bUp2, bDown2, reversed, 127);
	}
	else if(lift->type != NOPID || lift->type != DIFFERENTIAL){//INTAKE & DIFFERENTIAL is only type without PID
		if (abs(SensorValue[lift->sensor] - lift->goal) < lift->PID.thresh || abs(lift->velocity) < velLimit) {
			if (!lift->PID.isRunning) lift->goal = SensorValue[lift->sensor];//sets goal if not already running
				lift->PID.isRunning = true;//now pid is definitely running
		}
		else {
			lift->PID.isRunning = false;
			liftMove(lift, 0);//decelerate by turning off motors
		}
		PIDLift(lift);//calls the pid function for the lifts
	}
	else if (lift->type == NOPID) 		liftMove(lift, 0);
	else if (lift->type == DIFFERENTIAL)return;//dont even touch motors
	else return;
}
task fourBarPID(){
	for(;;){
		if(U5 || U5_2) {
			FourBar.PID.kP = 0.2;
			FourBar.PID.thresh = 60;
			FourBar.goal = FourBar.max;
			FourBar.PID.isRunning = true;
		}
		else if(D5 || D5_2) {
			FourBar.PID.kP = 0.8;
			FourBar.PID.thresh = 40;
			FourBar.goal = FourBar.min;
			FourBar.PID.isRunning = true;
		}
		delay(10);
		PIDLift(&FourBar);//calls the pid function for the lifts
	}
}
task LiftControlTask() {
	startTask(fourBarPID);
	for (;;) {//while true
		if(!autonRunning){
			if(U8 || D8 || U8_2 || D8_2){
				mainLift.PID.isRunning = false;
				LiftLift(&mogo, U8, D8, U8_2, D8_2, false,  180);
			}
			else LiftLift(&mainLift, U6, D6, U6_2, D6_2, false, 400);
			//	if(!autoStacking || !autonRunning) LiftLift(&goliat,	L8, R8, L8_2, R8_2, false);
		}
		else {
			PIDLift(&mainLift);//calls the pid function for the lifts
			PIDLift(&FourBar);//calls the pid function for the lifts
		}
		delay(10);
	}
}
#endif

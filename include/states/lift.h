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
float PolyReg(const float amnt){
	return(-11.237 * (amnt * amnt) + 286.05 * (amnt) + 261.23);
}
void liftMove(const struct liftMech* lift, const float speed) {
	float power = limitUpTo(127, speed);
	motor[lift->motors[0]] = power;//up is fast
	motor[lift->motors[1]] = -power;//up is fast
}
void resetPIDVals(const struct PIDs* pid) {
	pid->LastError = 0;
	pid->Integral = 0;
	pid->Derivative = 0;
}
float pidController(const struct PIDs* PIDtype, const int goal) {
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
void PIDLift(const struct liftMech* lift) {
	if (lift->PID.isRunning) liftMove(lift, pidController(lift->PID, lift->goal));//power the lift with its PID
	else resetPIDVals(lift->PID);//turn off the PID and reset values
	delay(lift->liftPIDelay);//delay a lil bit
}
void manualLiftControl(const struct liftMech* lift, int bUp, int bDown, int bUp2, int bDown2, bool reversed, int maxSpeed) {
	int dir = 1;
	if (reversed) dir = -1;
	bool upButton = (bUp == 1 || bUp2 == 1);//defining what is up button
	bool downButton = (bDown == 1 || bDown2 == 1);//defining what is down button
	if (!upButton && !downButton) liftMove(lift, 0);//not pressed any buttons
	else if ((SensorValue[lift->sensor] >= lift->max && (upButton))
		|| (SensorValue[lift->sensor] <= lift->min && (downButton)))//pressing buttons but surpassed limits
	liftMove(lift, 0);//power 0
	else if (upButton) liftMove(lift, dir * maxSpeed);//up max speed
	else if (downButton) liftMove(lift, dir * -maxSpeed);//down max speed
	else liftMove(lift, 0);
}
void LiftLift(const struct liftMech* lift, int bUp, int bDown, int bUp2, int bDown2, float velLimit) {
	if (bUp || bDown || bUp2 || bDown2) {
		lift->PID.isRunning = false;
		manualLiftControl(lift, bUp, bDown, bUp2, bDown2, false, 127);
	}
	else if(lift->type != BINARY){//BINARY IS ONLY TOP/BOTTOM
		if (abs(SensorValue[lift->sensor] - lift->goal) < 200 || abs(lift->velocity) < velLimit) {
			if (!lift->PID.isRunning) lift->goal = SensorValue[lift->sensor];//sets goal if not already running
				lift->PID.isRunning = true;//now pid is definitely running
		}
		else {
			lift->PID.isRunning = false;
			liftMove(lift, 0);
		}
		PIDLift(lift);//calls the pid function for the lifts
	}
}
void UpUntil(const struct liftMech* lift, int goal, int speed) {
	lift->PID.isRunning = false;
	while (SensorValue[lift->sensor] < goal) {//brings lift up to goal
		liftMove(lift, speed);
		if(stopAutoStack) return;
	}
	lift->goal = SensorValue[lift->sensor];//keeps lift in last position
	lift->PID.isRunning = true;//re-enables pid
	return;
}
void DownUntil(struct liftMech* lift, int goal, int speed) {
	lift->PID.isRunning = false;
	while (SensorValue[lift->sensor] > goal ) {//brings lift down to goal
		liftMove(lift, -speed);
		if(stopAutoStack) return;
	}
	lift->goal = SensorValue[lift->sensor];//keeps lift in last position
	lift->PID.isRunning = true;//re-enables pid
	return;
}
task LiftControlTask() {
	for (;;) {//while true
		if(!autonRunning){
			LiftLift(&MogoLift, U6, D6, U6_2, D6_2, 180);
			LiftLift(&FourBar, U5, D5, U5_2, D5_2, 1200);
		}
		else {
			MogoLift.PID.kP = 0.15;
			PIDLift(&MogoLift);//calls the pid function for the lifts
			PIDLift(&FourBar);//calls the pid function for the lifts
		}
		delay(10);
	}
}
#endif


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
void PIDLift(const struct liftMech* lift) {
	if (lift->PID.isRunning) mechMove(lift->m, pidController(lift->PID, lift->goal));//power the lift with its PID
	else resetPIDVals(lift->PID);//turn off the PID and reset values
	delay(lift->PIDelay);//delay a lil bit
}
void LiftLift(struct liftMech* lift, int bUp, int bDown, int bUp2, int bDown2, const float velLimit, const bool limited = true) {
	if (bUp || bDown || bUp2 || bDown2) {
		lift->PID.isRunning = false;
		if(limited)	limitMechControl(lift->m, bUp, bDown, bUp2, bDown2, lift->min, lift->max, false, 127);
		else			buttonMechControl(lift->m, bUp, bDown, bUp2, bDown2, false, 127);
	}
	else if (abs(SensorValue[lift->m.sensor] - lift->goal) < 200 || abs(lift->m.velocity) < velLimit) {
		if (!lift->PID.isRunning) lift->goal = SensorValue[lift->m.sensor];//sets goal if not already running
		lift->PID.isRunning = true;//now pid is definitely running
		PIDLift(lift);//calls the pid function for the lifts
	}
	else {
		lift->PID.isRunning = false;
		mechMove(lift->m, 0);
	}
	//PIDLift(lift);//calls the pid function for the lifts
}
void UpUntil(struct liftMech* lift, const int goal, const int speed) {
	lift->PID.isRunning = false;
	while (SensorValue[lift->m.sensor] < goal) {//brings lift up to goal
		mechMove(lift->m, speed);
	}
	lift->goal = SensorValue[lift->m.sensor];//keeps lift in last position
	lift->PID.isRunning = true;//re-enables pid
	return;
}
void DownUntil(struct liftMech* lift, const int goal, const int speed) {
	lift->PID.isRunning = false;
	while (SensorValue[lift->m.sensor] > goal) {//brings lift down to goal
		mechMove(lift->m, -speed);
	}
	lift->goal = SensorValue[lift->m.sensor];//keeps lift in last position
	lift->PID.isRunning = true;//re-enables pid
	return;
}

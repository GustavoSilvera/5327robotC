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
void PIDLift(const struct liftMech* lift) {
	if (lift->PID.isRunning) liftMove(lift, pidCompute(lift->PID, lift->goal));//power the lift with its PID
	else resetPIDVals(lift->PID);//turn off the PID and reset values
	delay(lift->liftPIDelay);//delay a lil bit
}
void UpUntil(const struct liftMech* lift, int goal, int speed = 127) {
	lift->PID.isRunning = false;
	while (SensorValue[lift->sensor] < goal) {//brings lift up to goal
		liftMove(lift, abs(speed));
		if(stopAutoStack) return;
	}
	lift->goal = SensorValue[lift->sensor];//keeps lift in last position
	lift->PID.isRunning = true;//re-enables pid
	return;
}
void DownUntil(struct liftMech* lift, int goal, int speed = 127) {
	lift->PID.isRunning = false;
	while (SensorValue[lift->sensor] > goal ) {//brings lift down to goal
		liftMove(lift, -abs(speed));
		if(stopAutoStack) return;
	}
	lift->goal = SensorValue[lift->sensor];//keeps lift in last position
	lift->PID.isRunning = true;//re-enables pid
	return;
}
void goUntil(struct liftMech* lift, int goal, int speed = 127){
	if(SensorValue[lift->sensor] < goal) UpUntil(lift, goal);
	else if(SensorValue[lift->sensor] > goal) DownUntil(lift, goal);
	else liftMove(lift, 0);
	return;
}
void manualLiftControl(const struct liftMech* lift, int bUp, int bDown, int bUp2, int bDown2, bool reversed, int maxSpeed) {
	int dir = 1;
	int power;
	if (reversed) 			(dir = -1);
	bool upButton 		=	(bUp == 1 || bUp2 == 1);//defining what is up button
	bool downButton 	=	(bDown == 1 || bDown2 == 1);//defining what is down button
	bool withinUpper 	=	(SensorValue[lift->sensor] <= lift->max);//within upper bound
	bool withinLower 	=	(SensorValue[lift->sensor] >= lift->min);//within lower bound
	if (!upButton && !downButton) power = 0;//not pressed any buttons
	else if ( (!withinUpper && upButton) || (!withinLower && downButton)) power = 0;//pressing buttons but !within bounds
	else if (upButton) 	 power = dir * maxSpeed;//up max speed
	else if (downButton) power = -dir * maxSpeed;//down max speed
	else 				 power = 0;//anything else? just kill it
	int goal = 0;//goal for binary lifts(max or min)
	if(lift->type == BINARY){
		const int decelZone = 50;//where lift should stop sending power to motor from its goal
		if(getSign(power) > 0) goal = lift->max - decelZone;
		else goal = lift->min + decelZone;
	}
	if(lift->type == DIFFERENTIAL) liftDiff(lift, power);//main lift and 4bar are normal-er
	else if (lift->type == BINARY) goUntil(lift, goal);//brings to goal
	else liftMove(lift, power);//mogo has motors going opposite speeds
}
void LiftLift(const struct liftMech* lift, int bUp, int bDown, int bUp2, int bDown2, float velLimit = 100) {
	if (bUp || bDown || bUp2 || bDown2) {
		lift->PID.isRunning = false;
		manualLiftControl(lift, bUp, bDown, bUp2, bDown2, false, 127);
	}
	else if(lift->type != INTAKE || lift->type != DIFFERENTIAL){//INTAKE & DIFFERENTIAL is only type without PID
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
	else if (lift->type == INTAKE) 		liftMove(lift, 0);
	else if (lift->type == DIFFERENTIAL)return;//dont even touch motors
}
task LiftControlTask() {
	for (;;) {//while true
		if(!autonRunning){
			LiftLift(&mainLift, U6, D6, U6_2, D6_2, 180);
			LiftLift(&mogo, 	U8, D8, U8_2, D8_2, 180);
			LiftLift(&FourBar,	U5, D5, U5_2, D5_2, 1200);
			LiftLift(&goliat,	L8, R8, L8_2, R8_2);
		}
		else {
			PIDLift(&mainLift);//calls the pid function for the lifts
			PIDLift(&FourBar);//calls the pid function for the lifts
		}
		delay(10);
	}
}
#endif

/********************************************************************************\
* .----------------.  .-----------------. .----------------.  .----------------. *
*| .--------------. || .--------------. || .--------------. || .--------------. |*
*| |     _____    | || | ____  _____  | || |     _____    | || |  _________   | |*
*| |    |_   _|   | || ||_   \|_   _| | || |    |_   _|   | || | |  _   _  |  | |*
*| |      | |     | || |  |   \ | |   | || |      | |     | || | |_/ | | \_|  | |*
*| |      | |     | || |  | |\ \| |   | || |      | |     | || |     | |      | |*
*| |     _| |_    | || | _| |_\   |_  | || |     _| |_    | || |    _| |_     | |*
*| |    |_____|   | || ||_____|\____| | || |    |_____|   | || |   |_____|    | |*
*| |              | || |              | || |              | || |              | |*
*| '--------------' || '--------------' || '--------------' || '--------------' |*
* '----------------'  '----------------'  '----------------'  '----------------' *
\********************************************************************************/
void initMech(struct apparatus* side, enum mechType t, char sensor, int motre1, int motre2) {
	side->type = t;
	side->speed = 0;
	side->m.sensor = sensor;
	side->m.motors[0] = motre1;
	side->m.motors[1] = motre2;//defaulted to 0
	side->m.velocity = 0.0;
	side->m.past = 0.0;
	side->m.stalling = false;
}
void initLift(struct liftMech* lift, char sensor, int motre, int motre2, int min, int max, int delayamnt = 20) {
	lift->min = min;
	lift->max = max;
	lift->PIDelay = delayamnt;
	lift->m.sensor = sensor;
	lift->m.motors[0] = motre;
	lift->m.motors[1] = motre2;
	lift->m.velocity = 0.0;
	lift->m.past = 0.0;
	lift->m.position = false;
	lift->goal = SensorValue[lift->m.sensor];

}
void initPID(struct PIDPar* PIDType, char sensor, int thresh, float kP, float kI, float kD, bool reversed, bool isRunning) {
	PIDType->sensor = sensor;
	PIDType->thresh = thresh;
	PIDType->kP = kP;//0.2 //pretty efficient lift tho 0.075
	PIDType->kI = kI;// 0 .05;//0.04;
	PIDType->kD = kD;//1;
	PIDType->LastError = 0;
	PIDType->Integral = 0;
	PIDType->reversed = reversed;
	PIDType->isRunning = isRunning;
}
void scaleGyros() {
	//Adjust SensorFullCount to set the "rollover" point. 3600 sets the rollover point to +/-3600
	//NO RESET
	SensorScale[Gyro] = 260;
	SensorFullCount[Gyro] = 36000;
}
void resetGyros() {
	SensorType[in6] = sensorNone;
	SensorType[in6] = sensorGyro;//resets gyro sensor, rly sketchy
	SensorValue[Gyro] = 0;//resets gyro sensor
	delay(300);
	scaleGyros();
}

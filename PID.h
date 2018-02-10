void resetPIDVals(struct PIDPar* pid) {
	pid->LastError = 0;
	pid->Integral = 0;
	pid->Derivative = 0;
}
float pidController(struct PIDPar* PIDtype, const float goal) {
	float error = SensorValue[PIDtype->sensor] - goal;//calculate error
	if (abs(error) < PIDtype->thresh) error = 0;
	const float untilIntegral = 20;//considered "low threshold" for potentiometers
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
	return dir * (PIDtype->kP * error + PIDtype->kI * PIDtype->Integral + PIDtype->kD * PIDtype->Derivative);
	//return(dir * getSign(error) * abs((PIDtype->kP * error) + (PIDtype->kI * PIDtype->Integral) + (PIDtype->kD * PIDtype->Derivative)));
}

/*******************************************************************************\
*.----------------.  .----------------.  .----------------.  .----------------.  *
*| .--------------. || .--------------. || .--------------. || .--------------. |*
*| | ____    ____ | || |  _________   | || |     ______   | || |  ____  ____  | |*
*| ||_   \  /   _|| || | |_   ___  |  | || |   .' ___  |  | || | |_   ||   _| | |*
*| |  |   \/   |  | || |   | |_  \_|  | || |  / .'   \_|  | || |   | |__| |   | |*
*| |  | |\  /| |  | || |   |  _|  _   | || |  | |         | || |   |  __  |   | |*
*| | _| |_\/_| |_ | || |  _| |___/ |  | || |  \ `.___.'\  | || |  _| |  | |_  | |*
*| ||_____||_____|| || | |_________|  | || |   `._____.'  | || | |____||____| | |*
*| |              | || |              | || |              | || |              | |*
*| '--------------' || '--------------' || '--------------' || '--------------' |*
*'----------------'  '----------------'  '----------------'  '----------------'  *
\*******************************************************************************/

float TruSpeed(const float value) {//for all other polynomials; visit: goo.gl/mhvbx4
	return(GETSIGN(value) * (SQUARE(value) / (127)));//squaring
}//function for calculating the truSpeed function based off a polynomial
void mechMove(const struct mechanism* mech, const int speed) {
	if(!mech->stalling){
		motor[mech->motors[0]] = speed;//up is fast
		motor[mech->motors[1]] = speed;//up is fast
	}
	else {
		motor[mech->motors[0]] = 0;//up is fast
		motor[mech->motors[1]] = 0;//up is fast
		delay(300);
	}
}
void buttonMechControl(const struct mechanism* mech, const int bUp, const int bDown, const int bUp2, const int bDown2, bool reversed, const int maxSpeed = 127) {
	int dir = 1;
	if (reversed) dir = -1;
	const bool upButton = (bUp == 1 || bUp2 == 1);//defining what is up button
	const bool downButton = (bDown == 1 || bDown2 == 1);//defining what is down button
	if (!upButton && !downButton) 	mechMove(mech, 0);//not pressed any buttons
	else if (upButton) 					mechMove(mech, dir * maxSpeed);//up max speed
	else if (downButton) 				mechMove(mech, dir * -maxSpeed);//down max speed
	else 										mechMove(mech, 0);
}
void analogMechControl(const struct mechanism* mech, const float power) {
	int speed = TruSpeed(power);
	if (autonRunning) {
		if (!mech->stalling) mechMove(mech, speed);
		else {
			mechMove(mech, 0);
			delay(750);//stall waiter
		}
	}
	else mechMove(mech, speed);//no antistall during manual
}
void limitMechControl(const struct mechanism* mech, int bUp, int bDown, int bUp2, int bDown2, int min, int max, bool reversed, int maxSpeed = 127) {
	const bool upButton = (bUp == 1 || bUp2 == 1);//defining what is up button
	const bool downButton = (bDown == 1 || bDown2 == 1);//defining what is down button
	if ((SensorValue[mech->sensor] <= max && upButton) || (SensorValue[mech->sensor] >= min && downButton)) {
		buttonMechControl(mech, bUp, bDown, bUp2, bDown2, reversed);
	}
	else mechMove(mech, 0);
}
task MechControlTask() {
	const float primary = 1;
	const float partner = 0.8;
	for (;;) {//while true
		buttonMechControl(&conveyer.m, U6, D6, U6_2, D6_2, false);
		//limitMechControl(&fourBar.m, U5, D5, U5_2, D5_2, FourBar.min, FourBar,nax, false);
		limitMechControl(&lock.m, U8, D8, U8_2, D8_2, lock.min, lock.max, false);
		//binaryMechControl(&lock.m, U8, lock.max, lock.min, false, 50);
		analogMechControl(&baseRight.m, primary*vexRT[Ch2] + partner*vexRT[Ch2Xmtr2]);
		analogMechControl(&baseLeft.m, primary*vexRT[Ch3] + partner*vexRT[Ch3Xmtr2]);
		delay(10);
	}
}

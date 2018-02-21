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
float calcVel(struct mechanism* mech, float dist, float delayAmount) {
	float velocity = LimitDownTo(1, ((SensorValue(mech->sensor) - mech->past) / dist) / ((float)(delayAmount / 1000)));//1000 ms in 1s;
	mech->past = SensorValue[mech->sensor];
	return(velocity);//1000 ms in 1s;
}
float calcRotVel() {
	float velocity = mRot - pastRot;
	pastRot = mRot;
	return(velocity);
}
task MeasureSpeed() {
	/*MEASURING IN IN/SEC*/
	float dist = 1.125*PI;
	float delayAmount = 50;
	for (;;) {
		//base & conveyer velocities
		baseRight.m.velocity = calcVel(&baseRight.m, circum, delayAmount);
		baseLeft.m.velocity = calcVel(&baseLeft.m, circum, delayAmount);
		//overall bot velocities
		mainVelocity = AVG(baseRight.m.velocity, baseLeft.m.velocity);
		rotVelocity = calcRotVel();
		//lift velocity
		lock.m.velocity = calcVel(&lock.m, dist, delayAmount);
		//does the waitings
		delay(delayAmount);
	}
}//task for measuring velocity of the base, in IN/Sec

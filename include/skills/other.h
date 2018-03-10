/********************************************************************************\
* .----------------.  .----------------.  .----------------.  .----------------. *
*| .--------------. || .--------------. || .--------------. || .--------------. |*
*| |     ____     | || |  _________   | || |  ____  ____  | || |  _______     | |*
*| |   .'    `.   | || | |  _   _  |  | || | |_   ||   _| | || | |_   __ \    | |*
*| |  /  .--.  \  | || | |_/ | | \_|  | || |   | |__| |   | || |   | |__) |   | |*
*| |  | |    | |  | || |     | |      | || |   |  __  |   | || |   |  __ /    | |*
*| |  \  `--'  /  | || |    _| |_     | || |  _| |  | |_  | || |  _| |  \ \_  | |*
*| |   `.____.'   | || |   |_____|    | || | |____||____| | || | |____| |___| | |*
*| |              | || |              | || |              | || |              | |*
*| '--------------' || '--------------' || '--------------' || '--------------' |*
* '----------------'  '----------------'  '----------------'  '----------------' *
\********************************************************************************/
//for the cool ascii text go here: http://patorjk.com/software/taag/#p=display&f=Blocks


task sensorsUpdate() {
	//int rot=0;
	for (;;) {
		mRot = ((float)(GyroK*SensorValue[Gyro]));
		encoderAvg = avg(SensorValue[RightBaseEnc], SensorValue[LeftBaseEnc]);
		delay(5);//really quick delay
	}
}
bool stalling(const struct mechanism* mech) {
	return (
	abs(motor[mech->motors[0]]) > 50 &&//high ish power
	abs(motor[mech->motors[1]]) > 50 &&//high ish power
	abs(mech->velocity) < 30//low ish velocity yet high speed (for like 500 ms)
	);
}
void checkStalling(struct mechanism* mech) {
	if (stalling(mech)) {
		clearTimer(T1);
		bool currentlyStalling = true;
		while (time1[T1] < 200) {//checkingn for continuous stalling (else instantanious refresh)
			currentlyStalling = stalling(mech);//still stalling
			if (currentlyStalling) continue;//keep going until time limit
			else break;
		}
		if (currentlyStalling)//done waiting, final check
			mech->stalling = true;//if so, consider it stalling
	}
	else mech->stalling = false;
	if (mech->stalling) playSound(soundBlip);
}

task antiStall() {
	for (;;) {
		checkStalling(&baseRight.m);
		checkStalling(&baseLeft.m);
		delay(50);
	}
}
task displayLCD(){
	for(;;){
		string encode, LeftSonar, RightSonar, gyroRead;
		//Display the Primary Robot battery voltage
		displayLCDString(0, 0, "B:");
		sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
		displayNextLCDString(mainBattery);
		displayNextLCDString(" ");
		//Display the encoder amnt
		displayNextLCDString("E:");
		sprintf(encode, "%d", (int)encoderAvg, 't');//Build the value to be displayed
		displayNextLCDString(encode);
		displayNextLCDString(" ");
		//NEXT LINE
		//Display the Left Sonar
		displayLCDString(1, 0, "L:");
		//sprintf(LeftSonar, "%d", SensorValue[Lsonar], '"');//Build the value to be displayed
		displayNextLCDString(LeftSonar);
		displayNextLCDString(" ");
		//Display the right Sonar
		displayLCDString(1, 5, "R:");
		//sprintf(RightSonar, "%d", SensorValue[Rsonar], '"');//Build the value to be displayed
		displayNextLCDString(RightSonar);
		displayNextLCDString(" ");
		//Display the Power Expander voltage
		displayLCDString(1, 10, "G:");
		sprintf(gyroRead, "%1.2f%c", SensorValue[Gyro] * GyroK);//Build the value to be displayed
		displayNextLCDString(gyroRead);
		delay(30);
	}
}
task killswitch() {
	for (;;) {
		if (D7 && autonRunning) {
			stopAllTasks();
		}
		delay(50);
	}
}

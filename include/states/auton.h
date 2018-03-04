#if !defined(AUTO_H)
#define AUTO_H
/*
.----------------.  .----------------.  .----------------.  .----------------.  .-----------------.
| .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
| |      __      | || | _____  _____ | || |  _________   | || |     ____     | || | ____  _____  | |
| |     /  \     | || ||_   _||_   _|| || | |  _   _  |  | || |   .'    `.   | || ||_   \|_   _| | |
| |    / /\ \    | || |  | |    | |  | || | |_/ | | \_|  | || |  /  .--.  \  | || |  |   \ | |   | |
| |   / ____ \   | || |  | '    ' |  | || |     | |      | || |  | |    | |  | || |  | |\ \| |   | |
| | _/ /    \ \_ | || |   \ `--' /   | || |    _| |_     | || |  \  `--'  /  | || | _| |_\   |_  | |
| ||____|  |____|| || |    `.__.'    | || |   |_____|    | || |   `.____.'   | || ||_____|\____| | |
| |              | || |              | || |              | || |              | || |              | |
| '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
'----------------'  '----------------'  '----------------'  '----------------'  '----------------'
*/
const int INTAKE = 100;//for roller hold
const int OUTTAKE = -127;
const int heightValues[15] = {
	2200, 2300, 2400, 2550, 2600, 2600,
	2750, 2850, 2940, 3030, 3120, 3210,
	3400, 3500, 3600 };//values for where the lift should go to when autoStacking
const int heightFourBar[15] = {
	FourBar.max, FourBar.max, FourBar.max, FourBar.max, FourBar.max,
	FourBar.max, FourBar.max, FourBar.max, FourBar.max, FourBar.max,
	FourBar.max - 250, FourBar.max - 150, FourBar.max - 250,
	FourBar.max - 350, FourBar.max - 500
};//values for where the lift should go to when autoStacking
const int heightStago[9] = {
	2780, 2970, 3050,
	3200, 3330, 3420,
	3540, 3750, 3900
};//values for where the lift should go to when autoStacking
const int delayValues[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 150, 200, 200, 200};//values for individual delays when autostacking
//const int delayValues[11] = {0, 0, 0, 0, 0, 0, 150, 150, 240, 200, 200};//values for individual delays when autstacking
volatile int intakeSpeed = 0;
void switchLEDs(){
	if(SensorValue[OddLED] == 1){
		SensorValue[OddLED] = 0;
		SensorValue[EvenLED] = 1;
	}
	else {
		SensorValue[OddLED] = 1;
		SensorValue[EvenLED] = 0;
	}
}
void waitTill(struct liftMech *lift, int goal, int thresh){
	clearTimer(T3);
	while(abs(SensorValue[lift->sensor] - goal) > thresh && time1[T3] < 1000){continue;}//wait until success
	return;
}
void standStack(int cone){
	autoStacking = true;
	intakeSpeed = INTAKE*0.8;
	float batteryScale = 1;//( 8.0 / nImmediateBatteryLevel );
	UpUntilW4Bar(heightValues[cone] * batteryScale + 50, 0.9, 127, true); //four bar up
	FourBar.goal = FourBar.max + 300;//keeps them there HARD
	waitTill(&FourBar, 3850, 100);
	delay(delayValues[cone]);
	DownUntil(&mainLift, SensorValue[mainLift.sensor] - 30, 127);
	intakeSpeed = OUTTAKE; //release cone
	liftMove(&mainLift, 0);
	delay(130);
	if(currentCone < 13){
		UpUntilW4Bar(limitUpTo(4090, SensorValue[mainLift.sensor] + 100), 0.875, 127, false);
		if(SensorValue[FourBar.sensor] > 2200)DownUntil(&FourBar, FourBar.min);//make sure four bar is down
		intakeSpeed = 0;
		DownUntil(&mainLift, mainLift.min + 500, 127);
		currentCone+=1;
		switchLEDs();
	}
	else intakeSpeed = 0;
	autoStacking = false;
}
void quickStack(int cone){
	autoStacking = true;
	intakeSpeed = INTAKE*0.8;
	UpUntilW4Bar(heightValues[cone] + 50, 0.95, 127, true);
	if(currentCone <= 13) FourBar.goal = FourBar.max;//keeps them there
	waitTill(&FourBar, FourBar.max, 100);
	delay(delayValues[cone]);
	//DownUntil(&mainLift, SensorValue[mainLift.sensor] - 150, 127);
	intakeSpeed = OUTTAKE; //release cone
	liftMove(&mainLift, 0); //stop lift
	delay(150);
	UpUntilW4Bar(limitUpTo(4090, SensorValue[mainLift.sensor] + 100), 0.75, 127, false); //make sure lift is above
	if(SensorValue[FourBar.sensor] > 2200)DownUntil(&FourBar, FourBar.min);//make sure four bar is down
	//DownUntil(&mainLift, 3100); //initial downUntil to start goliath intake later
	intakeSpeed = 0;
	DownUntil(&mainLift, mainLift.min + 500, 127);
	switchLEDs();
	//DownUntil(&mainLift, mainLift.min + 12200, 127);
	autoStacking = false;
	if(currentCone < 14) currentCone+=1;
}
void stack(int cc){
	autoStacking = true;
	//controls which stack to go to
	if(cc < 10 || cc == 14) standStack(currentCone);
	else quickStack(currentCone);
	autoStacking = false;
}
void matchStack(int cone){
	autoStacking = true;
	matchLoads = true;
	intakeSpeed = INTAKE;
	if(currentCone < 3) {
		mainLift.goal = 2650;
		mainLift.PID.isRunning = true;
		UpUntil(&FourBar, FourBar.max, 127);
		FourBar.goal = FourBar.max + 300;
		waitTill(&FourBar, FourBar.max-100, 100);
		mainLift.goal = 2000;
		delay(100);
		mainLift.goal = 2650;
		intakeSpeed = OUTTAKE; //drop cone without lift down
		delay(200);
		DownUntil(&FourBar, FourBar.min, 127);
		currentCone++;
	}
	else stack(currentCone);
	matchLoads = false;
	autoStacking = false;
}
task autoStack() {
	for (;;) {
		if (U7 ) stack(currentCone);
		//if (U7_2) chinaStrat(currentCone);
		if (R7_2) matchStack(currentCone);
		//if (goliat.stalling && !autoStacking) stack(currentCone);
		if ((D7 || L7_2) && time1[T2]>200) {
			currentCone = 0;//reset
			SensorValue[OddLED] = 0;
			SensorValue[EvenLED] = 0;
			playSound(soundException);
			clearTimer(T2);
		}
		if ((R7 || D7_2) && time1[T2]>200 && currentCone > 0) {
			currentCone -= 1; //subtract one cone if autostack missed
			switchLEDs();
			playSound(soundDownwardTones);
			clearTimer(T2);
		}
		if ((L7 || U7_2) && time1[T2]>200 && currentCone < 14) {
			currentCone += 1; //add one cone
			switchLEDs();
			playSound(soundFastUpwardTones);
			clearTimer(T2);
		}
		delay(30);
	}
}
task goliathControl(){
	for(;;){
		if(autoStacking ||
			autonRunning){
			motor[goliath] = intakeSpeed;
		}
		else{
			if(L8 || L8_2)       motor[goliath] = 127;
			else if (R8 || R8_2) motor[goliath] = -127;
			else                 motor[goliath] = 0;
		}
		delay(50);
	}
}
task killswitch(){
	for(;;){
		//if(R7 && autonRunning) stopAllTasks();
		if((R7) && autoStacking ) {//autostack killswitch
			stopTask(autoStack);
			autoStacking = false;
			playSound(soundBeepBeep);//killed autostack
			startTask(autoStack);
			delay(300);
		}
		delay(50);
	}
}
/*

void threeConeAuton(bool right, bool twenty){
	int dir  = 1;
	if(!right) dir = -1;
	autonRunning = true;
	mogoAndPreload();
	driveFor(1);
	FourBar.goal = FourBar.min;
	DownUntil(&mainLift, mainLift.min, 127); //go for next cone
	mainLift.goal = mainLift.min - 200;
	mainLift.PID.isRunning = true;
	intakeSpeed = INTAKE; //intake first cone
	delay(350);
	stack(2); //stack first cone
	FourBar.goal = FourBar.min;
	mainLift.goal = mainLift.min + 600;
	driveFor(1);
	DownUntil(&mainLift, mainLift.min, 80);
	mainLift.goal = mainLift.min-400;
	mainLift.PID.isRunning = true;
	intakeSpeed = INTAKE;
	delay(400);
	stack(3);//stack second cone

	if(!twenty){ //score in 10pt
		driveFor2(-62);//drive back
		mainLift.goal = mainLift.max; //lift lift
		mainLift.PID.isRunning = true;
		if (right) RSwingFor(-45);//swing out
		else LSwingFor(45);
		rotFor(dir * -90, 2); //rotate perp to 10pt pole
		delay(200);
		driveFor(4); //drive to 10pt
		//mainLift.PID.isRunning = false;
		clearTimer(T4);
		while(SensorValue[mogo.sensor] > mogo.min && time1[T4] < 600){ //mogo out
			liftDiff(&mogo, 127);
		}
		//mainLift.PID.isRunning = true;
		driveFor(-25);// release & get out of the way
	}
	else { //score in 20 pt
		driveFor2(-62);//drive back
		mainLift.goal = mainLift.max; //lift lift
		mainLift.PID.isRunning = true;
		if(right) RSwingFor(-45);
		else LSwingFor(45);
		driveFor(-6); //drive to center of zone
		rotFor(dir * -90 , 2);
		driveFor(-5);//reverse to gain momentum
		delay(100);
		fwds(127);
		delay(1500);
		fwds(0);
		//driveFor(16); //enter 20pt zone
		clearTimer(T4);
		while(SensorValue[mogo.sensor] > mogo.min + 100 && time1[T4] < 600){ //mogo out
			liftDiff(&mogo, 127);
		}
		driveFor(-17);//exit zones
	}
	autonRunning = false;
}
void oneConeAuton(bool right, bool twenty){
	int dir  = 1;
	if(!right) dir = -1;
	autonRunning = true;
	mogoAndPreload();
	if(!twenty){ //score in 10pt
		driveFor2(-60);//drive back
		mainLift.goal = mainLift.max; //lift lift
		mainLift.PID.isRunning = true;
		if (right) RSwingFor(-45);//swing out
		else LSwingFor(45);
		rotFor(dir * -90, 2); //rotate perp to 10pt pole
		delay(200);
		driveFor(4); //drive to 10pt
		//mainLift.PID.isRunning = false;
		clearTimer(T4);
		while(SensorValue[mogo.sensor] > mogo.min && time1[T4] < 600){ //mogo out
			liftDiff(&mogo, 127);
		}
		//mainLift.PID.isRunning = true;
		driveFor(-25);// release & get out of the way
	}
	else { //score in 20 pt
		driveFor2(-62);//drive back
		mainLift.goal = mainLift.max; //lift lift
		mainLift.PID.isRunning = true;
		if(right) RSwingFor(-45);
		else LSwingFor(45);
		driveFor(-6); //drive to center of zone
		rotFor(dir * -90 , 2);
		driveFor(-5);//reverse to gain momentum
		delay(100);
		fwds(127);
		delay(1500);
		fwds(0);
		//driveFor(16); //enter 20pt zone
		clearTimer(T4);
		while(SensorValue[mogo.sensor] > mogo.min + 100 && time1[T4] < 600){ //mogo out
			liftDiff(&mogo, 127);
		}
		driveFor(-17);//exit zones
	}
	autonRunning = false;
}
void matchLoadAuton (bool right){
	int dir  = 1;
	if(!right) dir = -1;
	autonRunning = true;
	mainLift.PID.kP = 0.15;
	startTask(mogoOut);
	FourBar.PID.isRunning = true;
	intakeSpeed = INTAKE/2;
	driveFor2(48);//48
	//FourBar.goal = FourBar.max;
	stopTask(mogoOut);
	fwds(0);
	clearTimer(T4);
	while(time1[T4] < 600){
		FourBar.goal = FourBar.max;
		liftDiff(&mogo, -127);
	}
	liftMove(&mogo, 0);
	intakeSpeed = OUTTAKE; //release preload
	driveFor2(-9);//drive back
	intakeSpeed = 0;
	mainLift.goal = 2800; //lift lift above loader height
	mainLift.PID.isRunning = true;
	if (right) RSwingFor(-90);//swing to face loader
	else LSwingFor(90);
	intakeSpeed = INTAKE;
	mainLift.goal = 2400;
	delay(100);
	matchStack(2);
}

*/
#endif

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
	2200, 2300, 2400, 2500, 2600, 2600,
	2700, 2800, 2900, 3000, 3100, 3200,
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
void chinaStrat(int cone){
	autoStacking = true;
	intakeSpeed = OUTTAKE/2;
	if(currentCone < 9) UpUntilW4Bar(heightValues[cone], 0.95, 127, true);
	else{
		UpUntil(&mainLift, heightValues[cone], 127);
		UpUntil(&FourBar, heightFourBar[cone], 100);
	}
	FourBar.goal = FourBar.max + 300;//keeps them there
	delay(delayValues[cone]);
	intakeSpeed = INTAKE;
	DownUntil(&mainLift, SensorValue[mainLift.sensor] - 200, 127);
	currentCone--;//minus one on the internal stack
	liftMove(&mainLift, 0);
	delay(350);
	UpUntilW4Bar(SensorValue[mainLift.sensor] + 120, 0.75, 127, false);
	delay(100);
	DownUntil(&mainLift, heightStago[currentStag], 80);
	intakeSpeed = OUTTAKE;
	delay(100);
	autoStacking = false;
	if(currentCone < 14) {
		currentStag++;
		switchLEDs();
	}
}
void standStack(int cone){
	autoStacking = true;
	intakeSpeed = INTAKE*0.8;
	float batteryScale = 1;//( 8.0 / nImmediateBatteryLevel );
	UpUntilW4Bar(heightValues[cone] * batteryScale, 0.79, 127, true); //four bar up
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
		intakeSpeed = INTAKE;
		DownUntil(&mainLift, mainLift.min + 500, 127);
		currentCone+=1;
		switchLEDs();
	}
	else intakeSpeed = INTAKE;
	autoStacking = false;
}
void quickStack(int cone){
	autoStacking = true;
	intakeSpeed = INTAKE*0.8;
	UpUntilW4Bar(heightValues[cone], 0.85, 127, true);
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
	intakeSpeed = INTAKE;
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
		FourBar.goal = FourBar.max + 500;
		waitTill(&FourBar, FourBar.max-100, 100);
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
	currentCone = 0;
	currentStag = 0;
	for (;;) {
		if (U7 ) stack(currentCone);
		if (U7_2) chinaStrat(currentCone);
		if (R7_2) matchStack(currentCone);
		if (goliat.stalling && !autoStacking && !matchLoads) stack(currentCone);
		if ((D7) && time1[T2]>200) {
			currentCone = 0;//reset
			SensorValue[OddLED] = 0;
			SensorValue[EvenLED] = 0;
			playSound(soundException);
			clearTimer(T2);
		}
		if ((R7) && time1[T2]>200 && currentCone > 0) {
			currentCone -= 1; //subtract one cone if autostack missed
			switchLEDs();
			playSound(soundDownwardTones);
			clearTimer(T2);
		}
		if ((L7) && time1[T2]>200 && currentCone < 15) {
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
		if(R7 && autonRunning) stopAllTasks();
		if((R7 || R7_2) && autoStacking ) {//autostack killswitch
			stopTask(autoStack);
			autoStacking = false;
			playSound(soundBeepBeep);//killed autostack
			startTask(autoStack);
			delay(300);
		}
		delay(50);
	}
}

#endif

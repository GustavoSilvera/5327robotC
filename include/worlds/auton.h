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
const int INSPEED = 100;//for roller hold
const int OUTSPEED = -127;
const int heightValues[15] = {
	2270, 2300, 2400, 2550, 2600, 2650,
	2790, 2850, 2910, 3050, 3120, 3210,
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
const int delayValues[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 150, 200, 250, 350};//values for individual delays when autostacking

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
int intakeSpeed = 0;
task outtakeGoliat(){
	while(autoStacking) liftMove(&goliat, intakeSpeed);
	liftMove(&goliat, 0);
	return;
}
volatile bool finalCone = false;//used if dont want autostack to complete the loop
void standStack(int cone){
	startTask(outtakeGoliat);
	intakeSpeed = 60;
	UpUntilW4Bar(heightValues[cone] - 50, 0.85, 127, true);
	FourBar.PID.goal = FourBar.max;//keeps them there
	waitTill(&FourBar, FourBar.max, 100);
	FourBar.PID.isRunning = false;
	intakeSpeed = -127;
	delay(delayValues[cone]);
	liftMoveT(&mainLift, -80, 130);
	delay(150);
	intakeSpeed = 0;
	if(!finalCone){
		FourBar.PID.isRunning = true;
		FourBar.PID.goal = FourBar.min;
		UpUntilW4Bar(limitUpTo(mainLift.max, SensorValue[mainLift.sensor] + 50), 0.8, 127, false); //make sure lift is above
		waitTill(&FourBar, FourBar.min + 500, 100);//waits till 4bar gets away from stack
		DownUntil(&mainLift, mainLift.min + 400, 127);
	}
	switchLEDs();
	if(currentCone < 14) currentCone+=1;
}
void standStackMom(int cone){
	startTask(outtakeGoliat);
	intakeSpeed = 60;
	FourBar.PID.isRunning = true;
	FourBar.PID.goal = FourBar.min;//keeps them there
	UpUntilW4Bar(limitUpTo(mainLift.max, heightValues[cone] + 120), 0.95, 127, true);
	FourBar.PID.goal = FourBar.max;//keeps them there
	liftMoveT(&mainLift, -127, 230);
	waitTill(&FourBar, FourBar.max - 50, 100);
	intakeSpeed = OUTSPEED;
	delay(delayValues[cone]);
	liftMoveT(&mainLift, -80, 130);
	delay(150);
	if(!finalCone){
		FourBar.PID.isRunning = true;
		FourBar.PID.goal = FourBar.min;
		UpUntilW4Bar(limitUpTo(mainLift.max, SensorValue[mainLift.sensor] + 120), 0.8, 127, false); //make sure lift is above
		intakeSpeed = 0;
		waitTill(&FourBar, FourBar.min + 500, 100);//waits till 4bar gets away from stack
		DownUntil(&mainLift, mainLift.min + 400, 127);
	}
	switchLEDs();
	if(currentCone < 14) currentCone+=1;
}
void quickStack(int cone){
	UpUntilW4Bar(heightValues[cone] + 50, 0.9, 127, true);
	FourBar.PID.goal = FourBar.max;//keeps them there
	waitTill(&FourBar, FourBar.max, 100);
	FourBar.PID.isRunning = false;
	startTask(outtakeGoliat);
	delay(delayValues[cone] + 150);
	UpUntilW4Bar(limitUpTo(mainLift.max, SensorValue[mainLift.sensor] + 150), 0.75, 127, false); //make sure lift is above
	FourBar.PID.isRunning = true;
	FourBar.PID.goal = FourBar.min;
	mainLift.PID.goal = mainLift.min + 100;
	mainLift.PID.isRunning = true;
	waitTill(&mainLift, mainLift.PID.goal, 100);
	switchLEDs();
	if(currentCone < 14) currentCone+=1;
}
void stack(int cc){
	autoStacking = true; //controls which stack to go to
	standStackMom(currentCone);
	//if(cc < 10 || cc == 14) standStack(currentCone);
	//else quickStack(currentCone);
	autoStacking = false;
}
task autoStack() {
	for (;;) {
		if (U7 ) {
			finalCone = false;
			stack(currentCone);
		}
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
task killswitch(){
	for(;;){
		if(D7 && autoStacking) {
			finalCone = true;
			playSound(soundFastUpwardTones);
			playSound(soundDownwardTones);
			playSound(soundUpwardTones);
		}
		if(R7 && autonRunning) stopAllTasks();
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

#endif

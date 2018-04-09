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
const int heightValues[16] = {
	2050, 2100, 2150, 2220, 2300, 2380,
	2490, 2590, 2700, 2800, 2920, 2990,
	3050, 3110, 3270, 3500 };//values for where the lift should go to when autoStacking
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
const int delayValues[15] = {50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 150, 150, 150};//values for individual delays when autostacking
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
task goliatTask(){
	while(autoStacking || autonRunning) liftMove(&goliat, intakeSpeed);
	liftMove(&goliat, 0);
	return;
}
volatile bool finalCone = false;//used if dont want autostack to complete the loop
void standStack(int cone){
	intakeSpeed = 60;
	UpUntilW4Bar(heightValues[cone], 0.89, 127, true);
	FourBar.PID.goal = FourBar.max;//keeps them there
	waitTill(&FourBar, FourBar.max, 100);
	FourBar.PID.isRunning = false;
	delay(delayValues[cone]);
	intakeSpeed = -127;
	liftMoveT(&mainLift, -80, 130);
	delay(100);
	liftMoveT(mainLift, 127, 80);
	intakeSpeed = 0;
	switchLEDs();
	if(currentCone < 14) currentCone+=1;
}
void stackUp(int cc){
	startTask(goliatTask);
	autoStacking = true; //controls which stack to go to
	standStack(currentCone);
	holdPower = 0;
	//if(cc < 10 || cc == 14) standStack(currentCone);
	//else quickStack(currentCone);
	autoStacking = false;
}

void stackDown(int cc){
	autoStacking = true; //controls which stack to go to
	startTask(goliatTask);
	intakeSpeed = -127;
	FourBar.PID.isRunning = true;
	FourBar.PID.goal = FourBar.max + 100;
	liftMoveT(&mainLift, 90, 200);
	FourBar.PID.isRunning = true;
	FourBar.PID.goal = FourBar.min;
	waitTill(&FourBar, FourBar.min + 300, 100);//waits till 4bar gets away from stack
	intakeSpeed = 0;
	mainLift.PID.goal = mainLift.min + 300;
	autoStacking = false;
}
void matchStack(int cone){
	startTask(goliatTask);
	autoStacking = true;
	intakeSpeed = 60;
	if(currentCone < 3) {
		mainLift.PID.goal = 2650;
		mainLift.PID.isRunning = true;
		UpUntil(&FourBar, FourBar.max, 127);
		FourBar.PID.goal = FourBar.max;
		waitTill(&FourBar, FourBar.max, 100);
		mainLift.PID.goal = 2000;
		delay(100);
		mainLift.PID.goal = 2600;
		intakeSpeed = OUTTAKE; //drop cone without lift down
		delay(200);
		DownUntil(&FourBar, FourBar.min, 127);
		currentCone++;
	}
	else {
		stackUp(cone);
		stackDown(cone);
	};
	matchLoads = false;
	autoStacking = false;
}
task autoStack() {
	for (;;) {
		if (U7 ) {
			stackDown(currentCone);
		}
		/*if(U7){
			autoStacking = true;
			matchStack(currentCone);
			autoStacking = false;
		}*/
		/*if (D7 ) {
			stackDown(currentCone);
		}*/
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

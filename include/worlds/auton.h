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
	1830, 1940, 2070, 2100, 2190, 2280,
	2420, 2530, 2690, 2780, 2960, 3000,
	3140, 3230, 3350, 3370 };//values for where the lift should go to when autoStacking
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
void waitTill(struct liftMech *lift, int goal, int thresh){
	clearTimer(T3);
	while(abs(SensorValue[lift->sensor] - goal) > thresh && time1[T3] < 1000){continue;}//wait until success
	return;
}
volatile bool finalCone = false;//used if dont want autostack to complete the loop
void standStack(int cone){
	intakeSpeed = 127;
	UpUntilW4Bar(heightValues[cone], 0.89, 127, true);
	playSound(soundBeepBeep);
	delay(100);
	FourBar.PID.goal = FourBar.max;//keeps them there
	FourBar.PID.isRunning = true;
	waitTill(&FourBar, FourBar.max, 100);
	FourBar.PID.isRunning = false;
	/*delay(delayValues[cone]);
	intakeSpeed = -127;
	liftMoveT(&mainLift, -80, 130);
	delay(100);
	liftMoveT(&mainLift, 127, 80);
	intakeSpeed = 0;*/
	switchLEDs();
	if(currentCone < 15) currentCone+=1;
}
void stackUp(){
	//if(!autonRunning) startTask(goliatTask);
	autoStacking = true; //controls which stack to go to
	standStack(currentCone);
	autoStacking = false;
}

void stackDown(){
	autoStacking = true; //controls which stack to go to
	//if(!autonRunning) startTask(goliatTask);
	intakeSpeed = -127;
	hasCone = false;
	FourBar.PID.isRunning = true;
	FourBar.PID.goal = FourBar.max + 100;
	delay(200);
	liftMoveT(&mainLift, 90, 200);

	FourBar.PID.isRunning = true;
	FourBar.PID.goal = FourBar.min;
	waitTill(&FourBar, FourBar.min + 300, 100);//waits till 4bar gets away from stack
	intakeSpeed = 0;
	mainLift.PID.goal = mainLift.min + 300;
	autoStacking = false;
}
void matchStack(int cone){
	//startTask(goliatTask);
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
		intakeSpeed = OUTSPEED; //drop cone without lift down
		delay(200);
		DownUntil(&FourBar, FourBar.min, 127);
		currentCone++;
	}
	else {
		stackUp();
		stackDown();
	};
	matchLoads = false;
	autoStacking = false;
}
task autoStack() {
	//startTask(goliatTask);
	for (;;) {
		if ((U7 || U7_2)) stackDown();

		/*if(U7){
			autoStacking = true;
			matchStack(currentCone);
			autoStacking = false;
		}*/
		/*if (D7 ) {
			stackDown(currentCone);
		}*/
		//if (goliat.stalling && !autoStacking) stack(currentCone);
		delay(30);
	}
}
#endif

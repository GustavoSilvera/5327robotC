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
void standStack(int cone){
	intakeSpeed = 127;
	UpUntilW4Bar(heightValues[cone], 0.89, 127, true);
	playSound(soundBeepBeep);
	delay(100);
	FourBar.PID.goal = FourBar.max;//keeps them there
	FourBar.PID.isRunning = true;
	waitTill(&FourBar, FourBar.max, 100);
	FourBar.PID.isRunning = false;
	delay(delayValues[cone]);
	intakeSpeed = -127;
	liftMoveT(&mainLift, -80, 130);
	delay(100);
	liftMoveT(&mainLift, 127, 80);
	intakeSpeed = 0;
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
	delay(100);
	liftMoveT(&mainLift, 127, 200);
	FourBar.PID.isRunning = true;
	DownUntil(&FourBar, FourBar.min, 127);
	FourBar.PID.goal = FourBar.min;
	waitTill(&FourBar, FourBar.min + 300, 100);//waits till 4bar gets away from stack
	intakeSpeed = 0;
	mainLift.PID.goal = mainLift.min + 300;
	autoStacking = false;
}
task autoStack() {
	//startTask(goliatTask);
	for (;;) {
		if ((U7 || U7_2)) stackDown();
		if(false) stackUp();//just to get rid of that anoying warning
		delay(30);
	}
}
#endif

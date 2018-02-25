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
	2350, 2400, 2480, 2560, 2670, 2780,
	2840, 2970, 3050, 3200, 3330, 3420,
	3540, 3750, 3900 };//values for where the lift should go to when autoStacking
const int heightFourBar[15] = {
	FourBar.max, FourBar.max, FourBar.max, FourBar.max, FourBar.max,
	FourBar.max, FourBar.max, FourBar.max, FourBar.max, FourBar.max,
	FourBar.max - 150, FourBar.max - 150, FourBar.max - 250,
	FourBar.max - 350, FourBar.max - 500
};//values for where the lift should go to when autoStacking
const int heightStago[9] = {
	2780, 2970, 3050,
	3200, 3330, 3420,
	3540, 3750, 3900
};//values for where the lift should go to when autoStacking
const int delayValues[15] = {0, 0, 0, 0, 0, 0, 0, 0, 200, 250, 300, 350, 400, 450, 500};//values for individual delays when autostacking
//const int delayValues[11] = {0, 0, 0, 0, 0, 0, 150, 150, 240, 200, 200};//values for individual delays when autstacking
volatile int intakeSpeed = 0;
task goliathHold(){
	for(;;){
		motor[goliath] = intakeSpeed;
		delay(50);
	}
}
void chinaStrat(int cone){
	if (!autonRunning) startTask(goliathHold);
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
	if(!autonRunning) stopTask(goliathHold);
	if(currentCone < 14) currentStag++;
}
void stack(int cone){
	if (!autonRunning)
		startTask(goliathHold);
	autoStacking = true;
	intakeSpeed = INTAKE;
	float batteryScale = 3 * ( 9 / nImmediateBatteryLevel );
	if(currentCone < 9) UpUntilW4Bar(heightValues[cone] + 90 * batteryScale, 0.95, 127, true);
	else{
		UpUntil(&mainLift, heightValues[cone], 127);
		UpUntil(&FourBar, heightFourBar[cone], 100);
	}
	FourBar.goal = FourBar.max + 200;//keeps them there
	delay(delayValues[cone]);
	DownUntil(&mainLift, SensorValue[mainLift.sensor] - 150, 127);
	intakeSpeed = OUTTAKE;
	liftMove(&mainLift, 0);
	delay(200);
	UpUntilW4Bar(SensorValue[mainLift.sensor] + 120, 0.75, 127, false);
	delay(100);
	DownUntil(&mainLift, mainLift.min + 900, 80);
	autoStacking = false;
	if(!autonRunning) stopTask(goliathHold);
	if(currentCone < 14) currentCone+=1;
}
task autoStack() {
	currentCone = 0;
	currentStag = 0;
	for (;;) {
		if (U7) stack(currentCone);
		if(U7_2) chinaStrat(currentCone);
		if ((D7) && !autoStacking && time1[T2]>200) {
			currentCone = 0;//reset
			playSound(soundException);
			clearTimer(T2);
		}
		if ((R7) && time1[T2]>200 && currentCone > 0) {
			currentCone -= 1; //subtract one cone if autostack missed
			playSound(soundDownwardTones);
			clearTimer(T2);
		}
		if ((L7) && time1[T2]>200 && currentCone < 15) {
			currentCone += 1; //add one cone
			playSound(soundFastUpwardTones);
			clearTimer(T2);
		}
		delay(30);
	}
}
task autoAutoStack(){
	while(!goliat.stalling)	{
		if(R8) return;
		motor[goliath] = 127;
	}
	stack(currentCone);
	return;
}
task killswitch(){
	for(;;){
		if(R7 && autonRunning){
			stopAllTasks();
		}
		if((D7 || D7_2) && autoStacking ) {//autostack killswitch
			stopTask(autoStack);
			stopTask(goliathHold);
			playSound(soundBeepBeep);//killed autostack
			startTask(autoStack);
			delay(300);
		}
		if (doubleTap()) startTask(autoAutoStack);
		delay(50);
	}
}

#endif

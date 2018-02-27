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
	2350, 2400, 2530, 2590, 2670, 2780,
	2840, 2970, 3050, 3200, 3350, 3475,
	3540, 3750, 3900 };//values for where the lift should go to when autoStacking
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
const int delayValues[15] = {0, 0, 0, 0, 0, 0, 0, 0, 200, 330, 300, 350, 400, 450, 500};//values for individual delays when autostacking
//const int delayValues[11] = {0, 0, 0, 0, 0, 0, 150, 150, 240, 200, 200};//values for individual delays when autstacking
volatile int intakeSpeed = 0;
task goliathControl(){
	for(;;){
		if(autoStacking ||
			autonRunning){
			motor[goliath] = intakeSpeed;
		}
		else{
			if(L8 || L8_2)		 motor[goliath] = 127;
			else if (R8 || R8_2) motor[goliath] = -80;
			else motor[goliath] = 0;
		}
		delay(50);
	}
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
	if(currentCone < 14) currentStag++;
}
void standStack(int cone){
	autoStacking = true;
	intakeSpeed = INTAKE;
	float batteryScale = 1;//3 * ( 9 / nImmediateBatteryLevel );
	if(currentCone < 9) UpUntilW4Bar(heightValues[cone] + 95 * batteryScale, 0.9, 127, true);
	else{
		UpUntil(&mainLift, heightValues[cone], 127);
		UpUntil(&FourBar, heightFourBar[cone], 127);
	}
	FourBar.goal = FourBar.max + 200;//keeps them there
	delay(delayValues[cone]);
	DownUntil(&mainLift, SensorValue[mainLift.sensor] - 30, 127);
	intakeSpeed = OUTTAKE;
	liftMove(&mainLift, 0);
	delay(180);
	if(currentCone < 13){
		UpUntilW4Bar(limitUpTo(4090, SensorValue[mainLift.sensor] + 100), 0.875, 127, false);
		//delay(100);
		mainLift.goal = mainLift.min + 100 * currentCone;
		//DownUntil(&mainLift, mainLift.min + 1000, 127);
		delay(100*currentCone);
		currentCone+=1;
	}
	else intakeSpeed = 0;
	autoStacking = false;
}
void quickStack(int cone){
	autoStacking = true;
	intakeSpeed = INTAKE;
	if(currentCone < 9) UpUntilW4Bar(heightValues[cone] - 240, 0.9, 127, true);
	else{
		UpUntil(&mainLift, heightValues[cone], 127);
		UpUntil(&FourBar, heightFourBar[cone], 100);
	}
	FourBar.goal = FourBar.max + 200;//keeps them there
	while(SensorValue[FourBar.sensor] < FourBar.max - 150){continue;}
	///delay(delayValues[cone]);
	//DownUntil(&mainLift, SensorValue[mainLift.sensor] - 150, 127);
	intakeSpeed = OUTTAKE;
	liftMove(&mainLift, 0);
	delay(200);
	UpUntilW4Bar(limitUpTo(4090, SensorValue[mainLift.sensor] + 70), 0.75, 127, false);
	delay(100);
	DownUntil(&mainLift, mainLift.min + 12200, 127);
	autoStacking = false;
	if(currentCone < 14) currentCone+=1;
}
void stack(int cc){
	//controls which stack to go to
	if(cc < 9 || cc == 14 || cc == 12) standStack(currentCone);
	else quickStack(currentCone);
}
task autoStack() {
	currentCone = 0;
	currentStag = 0;
	for (;;) {
		if (U7 ) stack(currentCone);
		if (U7_2) chinaStrat(currentCone);
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
	autoAutoStacking = true;
	stopTask(goliathControl);
	while(!goliat.stalling)	{
		if(R8 || U7) {
			startTask(goliathControl);
			autoAutoStacking = false;
			return;//cancel or manual autostack
		}
		motor[goliath] = 127;
	}
	autoAutoStacking = false;
	startTask(goliathControl);
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
			playSound(soundBeepBeep);//killed autostack
			startTask(autoStack);
			delay(300);
		}
		if (doubleTap()) startTask(autoAutoStack);
		delay(50);
	}
}

#endif

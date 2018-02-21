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
const int INTAKE = 70;//for roller hold
const int OUTTAKE = -127;
const int heightValues[15] = {
	2270, 2320, 2390, 2530, 2670, 2750,
	2840, 2970, 3050, 3200, 3330, 3420,
	3540, 3750, 3900};//values for where the lift should go to when autoStacking
const int heightFourBar[15] = {
	FourBar.max, FourBar.max, FourBar.max, FourBar.max, FourBar.max,
	FourBar.max, FourBar.max, FourBar.max, FourBar.max, FourBar.max,
	FourBar.max - 100, FourBar.max - 150, FourBar.max - 250,
	FourBar.max - 350, FourBar.max - 500
};//values for where the lift should go to when autoStacking
const int delayValues[15] = {0, 0, 0, 0, 0, 0, 0, 0, 200, 200, 200, 200, 200, 200, 400};//values for individual delays when autostacking
//const int delayValues[11] = {0, 0, 0, 0, 0, 0, 150, 150, 240, 200, 200};//values for individual delays when autstacking
volatile int intakeSpeed = 0;
task goliathHold(){
	for(;;){
		liftMove(&goliat, intakeSpeed);
		delay(50);
	}
}
void stack(int cone){
	//startTask(goliathHold); //don't start the task again?
	autoStacking = true;
	intakeSpeed = INTAKE;
	//if(currentCone <= 3) UpUntil(&mainLift, heightValues[currentCone]);
	//else UpUntilSonar(&mainLift);//brings lift up until sonar detects no obstruction
	if(currentCone < 9) UpUntilW4Bar(heightValues[cone], 0.95, 127, true);
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
	DownUntil(&mainLift, mainLift.min + 500, 80);
	autoStacking = false;
	//stopTask(goliathHold); //probably stops the first task
	currentCone+=1;
}
task autoStack() {
	currentCone = 0;
	for (;;) {
		if (U7 || U7_2) {
			stack(currentCone);
		}
		if ((D7 || D7_2) && !autoStacking) currentCone = 0;//reset
		if ((R7 || R7_2) && time1[T2]>100 && currentCone > 0) {
			currentCone -= 1; //subtract one cone if autostack missed
			playSound(soundDownwardTones);
			clearTimer(T2);
		}
		if ((L7 || L7_2) && time1[T2]>100 && currentCone < 15) {
			currentCone += 1; //add one cone
			playSound(soundUpwardTones);
			clearTimer(T2);
		}
		delay(30);
	}
}
int initMRot;
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
		delay(50);
	}
}
void kamakaze(){//ram auton
	fwds(127, mRot);
	delay(5000);
	fwds(0, mRot);
	return;
}
void twentyPointScore(const int dir){
	rotFor(-dir*30);
	mainLift.goal = 0.5*(mainLift.min + mainLift.max)+200;//gets lift up and out of way
	driveFor(-22);//-25 tho
	//position -135 degrees relative to starting position
	rotFor(-dir*90);//87 tho
	delay(200);
	//////////////MoGo.PID.kP = 0.05;
	//////////////MoGo.PID.isRunning = true;
	driveFor(6);
	//////////////MoGo.goal = 3000;
	fwds(90, mRot);//70
	delay(1000);
	driveFor(-15);
	//////////////MoGo.PID.kP = 0.15;
}
void fivePointScore(const int dir){
	rotFor(-dir*180);
	//////////////MoGo.goal = //////////////MoGo.min - 300;//bring out mogo & drive
	delay(1000);
	//////////////MoGo.PID.isRunning = false;
	driveFor(-10);
}
void threeConeAuton(const bool left){
	int dir = 1;//left autonv         anm
	if(left) dir = -1;
	autonRunning = true;
	//////////////MoGo.goal = //////////////MoGo.min - 400;//bring out mogo & drive
	//////////////MoGo.PID.isRunning = true;
	mainLift.goal = 0.5*(mainLift.max + mainLift.min) - 200;//bring up lift
	mainLift.PID.isRunning = true;
	FourBar.PID.kP = 0.01;//slow down four bar so cone doesn't fly out
	FourBar.goal = FourBar.min + 300;//1000;//four bar down
	FourBar.PID.isRunning = true;
	initMRot = mRot;
	delay(400);//wait for mogo to come out mostly
	driveFor(31);//49);
	delay(300);
	//PRELOAD (MOGO WITH CONE)
	//////////////MoGo.goal = //////////////MoGo.max;
	delay(650);
	DownUntil(&mainLift, mainLift.min + 50, 127);//brings lift down
	FourBar.PID.kP = 0.15;//return to normal kP value
	FourBar.goal = FourBar.min;
	//DownUntil(&FourBar, FourBar.min, 127);//bring FourBar down
	delay(200);
	//CONE 2
	mainLift.goal = mainLift.min + 400;

	UpUntil(&FourBar, FourBar.min + 400, 127);//brings lift up for next cone
	delay(200);
	driveFor(5);
	FourBar.goal = FourBar.min;
	DownUntil(&mainLift, mainLift.min, 127);//brings lift down (GRABBED CONE 1)
	delay(200);
	//FourBar.goal = FourBar.max;//brings up lift to prepare stack
	UpUntil(&mainLift, mainLift.min + 500, 127); //+300;
	UpUntil(&FourBar, FourBar.max, 127);
	delay(300);
	driveFor(2);
	DownUntil(&mainLift, mainLift.min, 127);//brings down lift
	delay(300);
	FourBar.goal = FourBar.min;//									(RELEASED CONE 1)
	delay(500);
	//UpUntil(&mainLift, mainLift.min + 300, 127);//brings lift up for next cone pickup

	//driveFor(2);
	mainLift.PID.isRunning = true;
	mainLift.goal = SensorValue[mainLift.sensor] + 200;
	delay(250);
	DownUntil(&FourBar, FourBar.min, 127);//ensures 4bar is down
	UpUntil(&mainLift, SensorValue[mainLift.sensor] + 200, 127);
	DownUntil(&mainLift, mainLift.min, 127);//								(GRABBED CONE 2)
	delay(300);
	UpUntil(&mainLift, mainLift.min + 400, 127);
	UpUntil(&FourBar, FourBar.max, 127);
	DownUntil(&mainLift, mainLift.min + 200, 127);
	FourBar.goal = FourBar.min; //														(RELEASED CONE 2)
	/*
	if(abs(initMRot - mRot) > 3){
	rot(getSign(initMRot - mRot)*dir*127);//checking direction if skewed too far
	delay(100);
	}*/


	mainLift.goal = mainLift.max;
	FourBar.goal = FourBar.max;//bring up FourBar and lift

	//simple placing
	/*
	driveFor(-5);
	//////////////MoGo.goal = //////////////MoGo.min-300;
	delay(1000);
	driveFor(-3);
	//////////////MoGo.PID.isRunning = false;*/

	//scoring in zone
	driveFor(-35);//-53
	//delay(2000);
	fivePointScore(dir);//twentyPointScore(dir);
	autonRunning = false;
	return;
}
void EZAuton(const bool left){
	int dir = 1;//left auton
	if(left) dir = -1;
	autonRunning = true;
	////////////////MoGo.PID.kP = 0.;
	//////////////MoGo.goal = //////////////MoGo.min-300;//bring out mogo & drive
	//////////////MoGo.PID.isRunning = true;
	mainLift.goal = 1400;//bring up lift
	mainLift.PID.isRunning = true;
	FourBar.PID.kP = 0.05;//slow down four bar so cone doesn't fly out
	FourBar.goal = FourBar.min;//four bar down
	FourBar.PID.isRunning = true;
	initMRot = mRot;
	////////////////MoGo.PID.kP
	delay(400);//wait for mogo to come out mostly
	driveFor(49);//49
	delay(300);
	//PRELOAD (MOGO WITH CONE)
	//////////////MoGo.goal = //////////////MoGo.max;
	delay(550);
	DownUntil(&mainLift, mainLift.min - 50, 127);//brings lift down
	FourBar.goal = 0.5*(FourBar.max + FourBar.min);//brings halfway
	delay(200);
	mainLift.goal = 0.5*(mainLift.min + mainLift.max);//brings halfway
	rotFor(-5);
	driveFor(-43);//-45
	twentyPointScore(dir);
	autonRunning = false;
	return;
}

#endif

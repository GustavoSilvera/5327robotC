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
const int heightValues[11] = {220, 220, 370, 500, 620, 740, 830, 900, 1100, 1250, 1400};//values for where the lift should go to when autoStacking
const int coneHeight = 150;//how much the lift goes up DOWN after reaching height values
const int delayValues[11] = {400, 0, 0, 0, 0, 0, 100, 150, 100, 0, 0};//values for individual delays when autostacking
//const int delayValues[11] = {0, 0, 0, 0, 0, 0, 150, 150, 240, 200, 200};//values for individual delays when autstacking
task autoStack() {
	for (;;) {
		if ((U7 || U7_2) && currentCone < 11) {
			autoStacking = true;
			stopAutoStack = false;
			FourBar.PID.kP = 0.15;
			MogoLift.PID.isRunning = false;
			FourBar.PID.isRunning = true;
			//brings four bar up to prevent cone hitting mogo
			FourBar.goal = 1700;//0.5*(FourBar.min + FourBar.max);//brings up a bit
			UpUntil(&FourBar, FourBar.min + 200, 127);
			delay(100);
			//brings lift up to value based on coneIndex
			UpUntil(&MogoLift, limitUpTo(MogoLift.max, heightValues[currentCone] + MogoLift.min + 100), 127);
			FourBar.PID.isRunning = false;
			//bring fourbar up
			delay(delayValues[currentCone] * 0.75);
			UpUntil(&FourBar, FourBar.max, 127);
			//keep fourbar up
			FourBar.goal = FourBar.max;
			FourBar.PID.isRunning = false;
			delay(delayValues[currentCone] * 0.9);
			//bring lift down
			if(currentCone == 0)	DownUntil(&MogoLift,MogoLift.min, 127);
			DownUntil(&MogoLift, heightValues[currentCone] + MogoLift.min - coneHeight, 127);
			//bring fourbar down
			FourBar.PID.kP = 0.35;//keeps in place
			FourBar.goal = FourBar.max + 50;
			if(currentCone == 0) delay(200);
			DownUntil(&FourBar, FourBar.min, 127);
			//	UpUntil(&FourBar, 1600, 127);//brings 4bar back up
			FourBar.PID.isRunning = true;
			FourBar.PID.kP = 0.15;
			if(!stopAutoStack) currentCone++;//assumes got cone
				autoStacking = false;
		}
		if (D7 || D7_2) currentCone = 0;//reset
			if ((R8 || R8_2) && time1[T2]>300 && currentCone > 0) {
			currentCone -= 1; //subtract one cone if autostack missed
			playSound(soundDownwardTones);
			clearTimer(T2);
		}
		if ((L8 || L8_2) && time1[T3]>300 && currentCone < 11) {
			currentCone += 1; //add one cone
			playSound(soundUpwardTones);
			clearTimer(T3);
		}
		//led stuff
		if(currentCone % 2 == 0) {
			SensorValue[EvenLED] = 1;//even led on
			SensorValue[OddLED] = 0;
		}
		else if (currentCone == 11){
			SensorValue[EvenLED] = 0;
			SensorValue[OddLED] = 1;
			delay(100);
			SensorValue[EvenLED] = 1;
			SensorValue[OddLED] = 0;
			delay(100);
		}
		else {
			SensorValue[EvenLED] = 0;
			SensorValue[OddLED] = 1;//odd led on
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
				stopAutoStack = true;
				stopTask(autoStack);
				playSound(soundBeepBeep);//killed autostack
				delay(100);
				startTask(autoStack);
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
		MogoLift.goal = 0.5*(MogoLift.min + MogoLift.max)+200;//gets lift up and out of way
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
		MogoLift.goal = 0.5*(MogoLift.max + MogoLift.min) - 200;//bring up lift
		MogoLift.PID.isRunning = true;
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
		DownUntil(&MogoLift, MogoLift.min + 50, 127);//brings lift down
		FourBar.PID.kP = 0.15;//return to normal kP value
		FourBar.goal = FourBar.min;
		//DownUntil(&FourBar, FourBar.min, 127);//bring fourbar down
		delay(200);
		//CONE 2
		MogoLift.goal = MogoLift.min + 400;

		UpUntil(&FourBar, FourBar.min + 400, 127);//brings lift up for next cone
		delay(200);
		driveFor(5);
		FourBar.goal = FourBar.min;
		DownUntil(&MogoLift, MogoLift.min, 127);//brings lift down (GRABBED CONE 1)
		delay(200);
		//FourBar.goal = FourBar.max;//brings up lift to prepare stack
		UpUntil(&MogoLift, MogoLift.min + 500, 127); //+300;
		UpUntil(&FourBar, FourBar.max, 127);
		delay(300);
		driveFor(2);
		DownUntil(&MogoLift, MogoLift.min, 127);//brings down lift
		delay(300);
		FourBar.goal = FourBar.min;//									(RELEASED CONE 1)
		delay(500);
		//UpUntil(&MogoLift, MogoLift.min + 300, 127);//brings lift up for next cone pickup

		//driveFor(2);
		MogoLift.PID.isRunning = true;
		MogoLift.goal = SensorValue[MogoLift.sensor[0]] + 200;
		delay(250);
		DownUntil(&FourBar, FourBar.min, 127);//ensures 4bar is down
		UpUntil(&MogoLift, SensorValue[MogoLift.sensor[0]] + 200, 127);
		DownUntil(&MogoLift, MogoLift.min, 127);//								(GRABBED CONE 2)
		delay(300);
		UpUntil(&MogoLift, MogoLift.min + 400, 127);
		UpUntil(&FourBar, FourBar.max, 127);
		DownUntil(&MogoLift, MogoLift.min + 200, 127);
		FourBar.goal = FourBar.min; //														(RELEASED CONE 2)
		/*
		if(abs(initMRot - mRot) > 3){
		rot(getSign(initMRot - mRot)*dir*127);//checking direction if skewed too far
		delay(100);
		}*/


		MogoLift.goal = MogoLift.max;
		FourBar.goal = FourBar.max;//bring up fourbar and lift

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
		MogoLift.goal = 1400;//bring up lift
		MogoLift.PID.isRunning = true;
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
		DownUntil(&MogoLift, MogoLift.min - 50, 127);//brings lift down
		FourBar.goal = 0.5*(FourBar.max + FourBar.min);//brings halfway
		delay(200);
		MogoLift.goal = 0.5*(MogoLift.min + MogoLift.max);//brings halfway
		rotFor(-5);
		driveFor(-43);//-45
		twentyPointScore(dir);
		autonRunning = false;
		return;
	}

#endif

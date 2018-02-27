//extern void settle();
void rot(float power) {//rotates base
	motor[RBaseFront] = power;
	motor[RBaseBack] = power;
	motor[LBaseFront] = -power;
	motor[LBaseBack] = -power;
}
void swingL(int power){
	motor[LBaseFront] = -power;
	motor[LBaseBack] = -power;
}
void swingR(int power){
	motor[RBaseFront] = power;
	motor[RBaseBack] = power;
}
void timeTurn(int target, int numMogos){//(TUNE)
	rot(sgn(target)*127);
	if(numMogos == 2){
		switch(abs(target)){
		case 45:
			delay(270);
			break;
		case 90:
			delay(430);
			break;
		}
	}
	else if (numMogos == 0){
		switch(abs(target)){
		case 45:
			delay(500);
			break;
		case 90:
			delay(630);
			break;
		}
	}
	rot(0);
	return;
}
void turn(int deg, int df){
	int speed = sgn(deg) * 127;
	float initialRot = mRot*GyroK;

	rot(speed);
	switch(abs(deg)){
		case 45:
			delay(250);
			break;
		case 90:
			delay(400);
			break;
	}
	motor[RBaseFront] = 0;
	motor[RBaseBack] = 0;
	motor[LBaseFront] = 0;
	motor[LBaseBack] = 0;

	float error = (mRot - initialRot);
	while (abs(error - deg) < df) {
		speed = sgn(error) * 75;
		motor[RBaseFront] = speed;
		motor[RBaseBack] = speed;
		motor[LBaseFront] = -speed;
		motor[LBaseBack] = -speed;
	}
}
void rotFor(float target){
	SensorValue[RightBaseEnc]= 0;
	SensorValue[LeftBaseEnc]= 0;
	gyroBase.isRunning = true;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	clearTimer(T2);
	while(abs(SensorValue[Gyro]*GyroK - target) > 3 && time1[T2] < 900){//2 dF
		rot( 0.35*pidController( & gyroBase, target/GyroK) );
	}
	int power;
	if(target < 0) power = 127;
	else power = 127;
	rot(sgn(mainVelocity) * power);//gives settle time
	delay(100);
	rot(0);
	gyroBase.isRunning = false;
	resetPIDVals(&gyroBase);
	settle();
	return;
}
void rotFast(int target){
	SensorValue[RightBaseEnc]= 0;
	SensorValue[LeftBaseEnc] = 0;///reset
	SensorValue[Gyro] = 0;//resets gyros
	if(abs(target) == 90){
		while(abs(SensorValue[LeftBaseEnc]) < 230){
			rot(sgn(target)*127);
		}
	}
	else if (abs(target) == 45){
		while(abs(SensorValue[LeftBaseEnc]) < 120){
			rot(sgn(target)*127);
		}
	}
	rot(-sgn(target)*127);
	delay(10);
	clearTimer(T2);
	while(abs(SensorValue[Gyro] - target) > 1 && time1[T2] < 500){
		while(SensorValue[Gyro]*GyroK < target) rot(20);
		while(SensorValue[Gyro]*GyroK > target) rot(-20);
		delay(10);
	}
}
void rotAcc(int target, float kP, int delayTime = 1200){
	SensorValue[RightBaseEnc]= 0;
	SensorValue[LeftBaseEnc]= 0;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	clearTimer(T2);
	while(time1[T2] < delayTime){
		rot(LimitDownTo(35, -0.2*kP*(SensorValue[Gyro]*GyroK - target)));
	}
	settle();
}
void rotAccFast(int target){
	SensorValue[RightBaseEnc]= 0;
	SensorValue[LeftBaseEnc]= 0;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	clearTimer(T2);
	while(abs(SensorValue[Gyro] - target) > 0){
		rot(target - SensorValue[Gyro]*GyroK );
	}
	rot(-sgn(target - SensorValue[Gyro]*GyroK )*127);//hard break
	delay(10);
	while(SensorValue[Gyro]*GyroK < target) rot(60);
	while(SensorValue[Gyro]*GyroK > target) rot(-60);
	settle();
}
void rotEnc(int angle){
	//SensorValue[Gyro]= 0;
	//SensorScale[Gyro]= 260;
	int initGyro = SensorValue[Gyro];
	SensorValue[RightBaseEnc]= 0;
	SensorValue[LeftBaseEnc]= 0;
	int delayTime = 700;
	//rightEnc degreesToTicks = 10.9956/
	//distance = (angle/360)*2*pi*7 //2*pi*r (arc length)
	//to ticks -> *114.5916
	//constant is 4.4563
	int goalTicks = 10.5 * angle;
	int thresh = 16;
	clearTimer(T2);
	while(abs(SensorValue[RightBaseEnc] - goalTicks) > thresh){
		rot(getSign(angle)*127);
	}
	clearTimer(T2);
	while(time1[T2] < delayTime){
		rot(LimitDownTo(30, -2.5*((SensorValue[Gyro]-initGyro)*GyroK - angle)));
	}
	rot(0);
}
void rotTune(int angle){
	int initGyro = SensorValue[Gyro];
	int delayTime = 700;
	clearTimer(T2);
	while(time1[T2] < delayTime){
		rot(LimitDownTo(30, -1*((SensorValue[Gyro]-initGyro)*GyroK - angle)));
	}
}
void RSwingFor(int target){
	gyroBase.isRunning = true;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	clearTimer(T2);
	while(abs(SensorValue[Gyro]*GyroK - target) > 0.5 && time1[T2] < 500){
		swingR(LimitDownTo(20, 5*(target - SensorValue[Gyro]*GyroK) ) );
	}
	//swingR(-GETSIGN(target)*60);
	//delay(30);
	swingR(0);
	return;
}
void LSwingFor(int target){
	gyroBase.isRunning = true;
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	while(abs(SensorValue[Gyro]*GyroK - target) > 0.5 && time1[T2] < 500){
		//if (target>0) baseMove(&Right, 25); //keep right side still moving back
		swingL(LimitDownTo(20, 5*(SensorValue[Gyro]*GyroK- target) ) );
	}
	//swingL(-GETSIGN(target) *60);
	//delay(30);
	swingL(0);
	return;
}

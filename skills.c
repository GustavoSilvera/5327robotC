#pragma config(Sensor, in1,    RLin,           sensorLineFollower)
#pragma config(Sensor, in2,    LLin,           sensorLineFollower)
#pragma config(Sensor, in3,    LockPot,        sensorPotentiometer)
#pragma config(Sensor, in4,    FourBarPot,     sensorPotentiometer)
#pragma config(Sensor, in5,    BATERY_2_PORT,  sensorAnalog)
#pragma config(Sensor, in6,    Gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  LeftBaseEnc,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  RightBaseEnc,   sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  MogoFront,      sensorTouch)
#pragma config(Sensor, dgtl6,  MogoEnd,        sensorTouch)
#pragma config(Sensor, dgtl8,  sonar,          sensorSONAR_cm)
#pragma config(Motor,  port2,           ClawMotor,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           RBaseFront,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           RBaseBack,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           RFourBarMotor, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           ConveyorMotor, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           LBaseFront,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           LBaseBack,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           LFourBarMotor, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          LockMotor,     tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// This code is for the VEX cortex platform
#pragma platform(VEX2)
// Select Download method as "competition"
#pragma competitionControl(Competition)
//Main competition background code...do not modify!5
#include "Vex_Competition_Includes.c"
#include "randomstuff.c"
#define PI 3.1415

#define LEFT true
#define RIGHT false

#define SQUARE(x) (x * x)
#define AVG(x,y) ((x + y)/2)
#define MIN(x,y) (x <= y ? x : y)
#define GETSIGN(x) (x < 0 ? -1 : 1)
//#define LimitUpTo(x,y) (y < x ? y : x)
//#define LIMITDOWN(x,y) (y > x ? x : y)

#define INTAKE 127
#define OUTTAKE -127
#define OPEN 1, 0
#define CLOSE 0, 1
#define RELEASE fourBar.min
#define PICKUP  fourBar.max
#define UNLOCK 	UpUntil(&lock, lock.max, 127)
#define LOCK DownUntil(&lock, lock.min, 127)



const float circum = 4 * PI;//4 inch wheels

struct PIDPar {
	char sensor;
	volatile int forceDirection;//changing with multiple threads
	int motor1, motor2, thresh;
	float kP, kI, kD;
	bool reversed, Mopposite;
	volatile bool isRunning;
	float Integral, Derivative, LastError;
};
struct PIDPar gyroBase;
struct mechanism {//basic underlying mechanism class
	int motors[2];
	char sensor;
	bool stalling;
	float velocity, past;
	bool position;//for binary mechanisms
};
enum mechType { CONVEYER, DRIVE };//what kinds of lift we have
struct apparatus {
	enum mechType type;
	struct mechanism m;
	volatile float speed;//for multitasking
};
struct apparatus conveyer;
struct apparatus baseRight;
struct apparatus baseLeft;
struct liftMech {
	struct mechanism m;
	int min, max, PIDelay;
	struct PIDPar PID;
	volatile float goal;
};
struct liftMech lock;
struct liftMech fourBar;
//other
volatile float mainVelocity = 0;
volatile float rotVelocity = 0;
volatile float pastRot;
static int currentAutonomous = 0;
static const float GyroK = 15.0/80.0;//scales to normal +-360 degrees
volatile bool autonRunning = false;
string mainBattery, powerExpander;
//int startRot = 90;
volatile float mRot = 0.0;//current rotation
//volatile float encoderAvg;//used only for straight fwds and bkwds direction
//MISC FUNCTIONS

int getSign(int check) {
	if (check < 0) return -1;
	else if (check > 0) return 1;
	return 0;
}
float LimitUpTo(float max, float val) {
	if (abs(val) < abs(max)) return val;
	else return getSign(val) * max;
}
float LimitDownTo(float min, float val) {
	if (abs(val) > abs(min)) return val;
	else return getSign(val) * min;
}
float sqr(float val) {
	return val*val;
}
float avg(float a, float b) {
	return 0.5*(a + b);//avg between two things
}
/********************************************************************************\
* .----------------.  .-----------------. .----------------.  .----------------. *
*| .--------------. || .--------------. || .--------------. || .--------------. |*
*| |     _____    | || | ____  _____  | || |     _____    | || |  _________   | |*
*| |    |_   _|   | || ||_   \|_   _| | || |    |_   _|   | || | |  _   _  |  | |*
*| |      | |     | || |  |   \ | |   | || |      | |     | || | |_/ | | \_|  | |*
*| |      | |     | || |  | |\ \| |   | || |      | |     | || |     | |      | |*
*| |     _| |_    | || | _| |_\   |_  | || |     _| |_    | || |    _| |_     | |*
*| |    |_____|   | || ||_____|\____| | || |    |_____|   | || |   |_____|    | |*
*| |              | || |              | || |              | || |              | |*
*| '--------------' || '--------------' || '--------------' || '--------------' |*
* '----------------'  '----------------'  '----------------'  '----------------' *
\********************************************************************************/
void initMech(struct apparatus* side, enum mechType t, char sensor, int motre1, int motre2) {
	side->type = t;
	side->speed = 0;
	side->m.sensor = sensor;
	side->m.motors[0] = motre1;
	side->m.motors[1] = motre2;//defaulted to 0
	side->m.velocity = 0.0;
	side->m.past = 0.0;
	side->m.stalling = false;
}
void initLift(struct liftMech* lift, char sensor, int motre, int motre2, int min, int max, int delayamnt = 20) {
	lift->min = min;
	lift->max = max;
	lift->PIDelay = delayamnt;
	lift->m.sensor = sensor;
	lift->m.motors[0] = motre;
	lift->m.motors[1] = motre2;
	lift->m.velocity = 0.0;
	lift->m.past = 0.0;
	lift->m.position = false;
	lift->goal = SensorValue[lift->m.sensor];

}
void initPID(struct PIDPar* PIDType, char sensor, int thresh, float kP, float kI, float kD, bool reversed, bool isRunning) {
	PIDType->sensor = sensor;
	PIDType->thresh = thresh;
	PIDType->kP = kP;//0.2 //pretty efficient lift tho 0.075
	PIDType->kI = kI;// 0 .05;//0.04;
	PIDType->kD = kD;//1;
	PIDType->LastError = 0;
	PIDType->Integral = 0;
	PIDType->reversed = reversed;
	PIDType->isRunning = isRunning;
}
void scaleGyros() {
	//Adjust SensorFullCount to set the "rollover" point. 3600 sets the rollover point to +/-3600
	//NO RESET
	SensorScale[Gyro] = 260;
	SensorFullCount[Gyro] = 36000;
}
void resetGyros() {
	SensorType[in6] = sensorNone;
	SensorType[in6] = sensorGyro;//resets gyro sensor, rly sketchy
	SensorValue[Gyro] = 0;//resets gyro sensor
	delay(300);
	scaleGyros();
}
void initializeOpControl(const bool driver) {
	clearLCDLine(0);
	clearLCDLine(1);
	if (driver) resetGyros();//wastes time on auton
		mainVelocity = 0.0;
	//--------&reference-------TYPE-----------sensor-----------motor1---------motor2------------motor3-----(min, max)
	initMech( &conveyer,	 CONVEYER,    0,		ConveyorMotor,		0 	);//CONVEYOR
	initMech( &baseLeft,	 DRIVE,	      	LeftBaseEnc,		LBaseFront,		  LBaseBack);//LEFT BASE
	initMech( &baseRight,	 DRIVE,		  	RightBaseEnc,		RBaseFront,		  RBaseBack);//RIGHT BASE
	initLift( &lock,								LockPot,			LockMotor, 			0,						1000, 2000);//(min) && (max)
	initLift( &fourBar,							FourBarPot,		RFourBarMotor,		LFourBarMotor,		1300,	3500, 10);
	//--------&reference-------Sensor------thresh---kP---kI---kD---reversed---isRunning
	initPID ( &fourBar.PID, fourBar.m.sensor, 50, 0.05,  0.0, 0.05, 	true, 		true);
	initPID ( &lock.PID,		lock.m.sensor, 		30, 0.5, 	 0.0, 0.0, 		true, 		true);
	initPID ( &gyroBase, 		Gyro, 						1,  1, 0.0, 0, 		true, 		false);//kP = .35, kD = 0.6
	pastRot = mRot;
}
void resetPIDVals(struct PIDPar* pid) {
	pid->LastError = 0;
	pid->Integral = 0;
	pid->Derivative = 0;
}
float pidController(struct PIDPar* PIDtype, const float goal) {
	float error = SensorValue[PIDtype->sensor] - goal;//calculate error
	if (abs(error) < PIDtype->thresh) error = 0;
	const float untilIntegral = 20;//considered "low threshold" for potentiometers
	if (PIDtype->kI != 0) {//calculates integral (only at very end)
		if (abs(error) < untilIntegral) PIDtype->Integral += error;//used for averaging the integral amount, later in motor power divided by 25
		else PIDtype->Integral = 0;
	}
	else PIDtype->Integral = 0;
	// calculate the derivative
	PIDtype->Derivative = error - PIDtype->LastError;//change in errors
	PIDtype->LastError = error;
	// calculate drive (in this case, just for the lifts)
	int dir = 1;
	if (PIDtype->reversed) dir = -1;
	return dir * (PIDtype->kP * error + PIDtype->kI * PIDtype->Integral + PIDtype->kD * PIDtype->Derivative);
	//return(dir * getSign(error) * abs((PIDtype->kP * error) + (PIDtype->kI * PIDtype->Integral) + (PIDtype->kD * PIDtype->Derivative)));
}
//function for driving the robot
/*******************************************************************************\
*.----------------.  .----------------.  .----------------.  .----------------.  *
*| .--------------. || .--------------. || .--------------. || .--------------. |*
*| | ____    ____ | || |  _________   | || |     ______   | || |  ____  ____  | |*
*| ||_   \  /   _|| || | |_   ___  |  | || |   .' ___  |  | || | |_   ||   _| | |*
*| |  |   \/   |  | || |   | |_  \_|  | || |  / .'   \_|  | || |   | |__| |   | |*
*| |  | |\  /| |  | || |   |  _|  _   | || |  | |         | || |   |  __  |   | |*
*| | _| |_\/_| |_ | || |  _| |___/ |  | || |  \ `.___.'\  | || |  _| |  | |_  | |*
*| ||_____||_____|| || | |_________|  | || |   `._____.'  | || | |____||____| | |*
*| |              | || |              | || |              | || |              | |*
*| '--------------' || '--------------' || '--------------' || '--------------' |*
*'----------------'  '----------------'  '----------------'  '----------------'  *
\*******************************************************************************/
float TruSpeed(const float value) {//for all other polynomials; visit: goo.gl/mhvbx4
	return(GETSIGN(value) * (SQUARE(value) / (127));//squaring
}//function for calculating the truSpeed function based off a polynomial
void mechMove(const struct mechanism* mech, const int speed) {
	if(!mech->stalling){
		motor[mech->motors[0]] = speed;//up is fast
		motor[mech->motors[1]] = speed;//up is fast
	}
	else {
		motor[mech->motors[0]] = 0;//up is fast
		motor[mech->motors[1]] = 0;//up is fast
		delay(300);
	}
}
void buttonMechControl(const struct mechanism* mech, const int bUp, const int bDown, const int bUp2, const int bDown2, bool reversed, const int maxSpeed = 127) {
	int dir = 1;
	if (reversed) dir = -1;
	const bool upButton = (bUp == 1 || bUp2 == 1);//defining what is up button
	const bool downButton = (bDown == 1 || bDown2 == 1);//defining what is down button
	if (!upButton && !downButton) 	mechMove(mech, 0);//not pressed any buttons
	else if (upButton) 					mechMove(mech, dir * maxSpeed);//up max speed
	else if (downButton) 				mechMove(mech, dir * -maxSpeed);//down max speed
	else 										mechMove(mech, 0);
}
void analogMechControl(const struct mechanism* mech, const float power) {
	int speed = TruSpeed(power);
	if (autonRunning) {
		if (!mech->stalling) mechMove(mech, speed);
		else {
			mechMove(mech, 0);
			delay(750);//stall waiter
		}
	}
	else mechMove(mech, speed);//no antistall during manual
}
void limitMechControl(const struct mechanism* mech, int bUp, int bDown, int bUp2, int bDown2, int min, int max, bool reversed, int maxSpeed = 127) {
	const bool upButton = (bUp == 1 || bUp2 == 1);//defining what is up button
	const bool downButton = (bDown == 1 || bDown2 == 1);//defining what is down button
	if ((SensorValue[mech->sensor] <= max && upButton) || (SensorValue[mech->sensor] >= min && downButton)) {
		buttonMechControl(mech, bUp, bDown, bUp2, bDown2, reversed);
	}
	else mechMove(mech, 0);
}
void fwdsLong(const int power, const float angle = mRot) {//drive base forwards
	const float speed = LimitUpTo(127, power);
	//const float scalar = 0;//scalar for rotation (CHANGED)
	//float dirSkew = LimitUpTo(speed, scalar*(mRot - angle));
	motor[RBaseFront] = speed;
	motor[RBaseBack] = speed;
	motor[LBaseFront] = speed*0.625;
	motor[LBaseBack] = speed*0.625;
	//analogMechControl(&baseLeft.m, speed + dirSkew);
	//analogMechControl(&baseRight.m, speed - dirSkew);
}

void fwds(const int power, const float angle = mRot) {//drive base forwards
	const float speed = LimitUpTo(127, power);
	motor[RBaseFront] = speed;
	motor[RBaseBack] = speed;
	motor[LBaseFront] = speed;
	motor[LBaseBack] = speed;
}
void rot(float power) {//rotates base

	//analogMechControl(&baseRight.m, power);
	//analogMechControl(&baseLeft.m, -power);
	motor[RBaseFront] = power;
	motor[RBaseBack] = power;
	motor[LBaseFront] = -power;
	motor[LBaseBack] = -power;
}
void swingR(float power) {//swings right base
	analogMechControl(&baseRight.m, power);
}
void swingL(float power) {//swings right base
	analogMechControl(&baseRight.m, power);
}

void turn(int deg, int df){
	int speed = GETSIGN(deg) * 127;
	float initialRot = mRot*GyroK;

	motor[RBaseFront] = speed;
	motor[RBaseBack] = speed;
	motor[LBaseFront] = -speed;
	motor[LBaseBack] = -speed;

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
		speed = GETSIGN(error) * 75;
		motor[RBaseFront] = speed;
		motor[RBaseBack] = speed;
		motor[LBaseFront] = -speed;
		motor[LBaseBack] = -speed;
	}
}
bool nearStopped(int velThresh, int motorThresh){
	return(abs(mainVelocity) < velThresh && //low base velocity
		    abs(motor[baseRight.m.motors[0]]) < motorThresh && //low motor powers
		    abs(motor[baseLeft.m.motors[0]])  < motorThresh &&
		    abs(motor[baseRight.m.motors[1]]) < motorThresh &&
		    abs(motor[baseLeft.m.motors[1]])  < motorThresh
	);
}
void settle(){
	while(!nearStopped(30, 20)){//threshold for waiting
		fwds(0, mRot);
	}
	return;
}
void driveFor(float goal) {//drives for certain inches
	SensorValue[LeftBaseEnc] = 0;
	SensorValue[RightBaseEnc] = 0;
	const int thresh = 5;//10 ticks
	const int initDir = mRot;
	//ClearTimer(T1);
	//const float encoderScale = 1;//number of motor rotations = this() rotations
	const float dP = 20;//25;//multiplier for velocity controller
	//while (abs(goal * circum - encoderAvg) > thresh) {
	if(goal < 40){
		while (abs(goal * circum - SensorValue[LeftBaseEnc]*0.75) > thresh) {
			fwds(LimitDownTo(15, dP * ((goal*circum - SensorValue[LeftBaseEnc]*0.75 - 0.3*mainVelocity))), mRot);//initDir);
		}
	}
	else{
		while (abs(goal * circum - SensorValue[LeftBaseEnc]*0.75) > thresh) {
			fwdsLong(LimitDownTo(15, dP * ((goal*circum - SensorValue[LeftBaseEnc]*0.75 - 0.3*mainVelocity))), mRot);//initDir);
		}
	}

	fwds(0, initDir);
	settle();
	return;
}
void alignToLine(float dir){
	const float lineThresh = 1000;
	bool isAligned = (SensorValue[RLin] + SensorValue[LLin] < lineThresh);
	const int power = dir * 65;
	while(!isAligned){
		if(SensorValue[LLin] < lineThresh){
			fwds(0, mRot);
			while(SensorValue[RLin] > lineThresh){
				swingR(power);
			}
			isAligned = true;
		}
		else if (SensorValue[RLin] < lineThresh){
			fwds(0, mRot);
			while(SensorValue[LLin] > lineThresh){
				swingL(power);
			}
			isAligned = true;
		}
		else fwds(dir*60, mRot);// fwds(power, mRot); //speed where robot will stop on line
	}
	fwds(0, mRot);
}
void untilLine(int power){
	const float lineThresh = 1000;
	while((SensorValue[LLin] > lineThresh) && (SensorValue[RLin] > lineThresh)){
		fwds(power, mRot);
	}
	fwds(0, mRot);
}
void alignSonar(int goal){ //use sonar to reach distance in inches
	while(SensorValue[sonar] > goal){
		fwds(127, mRot);
	}
	fwds(-127, mRot);
	delay(50);
}
void rotFor(float target){
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
	rot(getSign(mainVelocity) * power);//gives settle time
	delay(100);
	rot(0);
	gyroBase.isRunning = false;
	resetPIDVals(&gyroBase);
	settle();
	return;
}
void rotEnc(int target){
	SensorValue[LeftBaseEnc] = 0;///reset
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	delay(100);
	if(abs(target) == 90){
		while(abs(SensorValue[LeftBaseEnc]) < 250){
			rot(getSign(target)*127);
		}
		rot(-getSign(target)*127);
		delay(100);
		while(SensorValue[Gyro]*GyroK < target) rot(60);
		while(SensorValue[Gyro]*GyroK > target) rot(-60);
	}
}
void rotAcc(int target, int delayTime = 1200){
	SensorValue[LeftBaseEnc] = 0;///reset
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	clearTimer(T2);
	while(time1[T2] < delayTime){
		rot(LimitDownTo(10, -5*(SensorValue[Gyro]*GyroK - target)));
	}
	settle();
}
void clawControl(int close, int open){
	if (open) {
		motor[ClawMotor] = 127;
		delay(700);
	}
	else if (close){
		motor[ClawMotor] = -80;
		delay(200);
	}
	else motor[ClawMotor] = 0;
}
task MechControlTask() {
	const float primary = 1;
	const float partner = 0.8;
	for (;;) {//while true
		buttonMechControl(&conveyer.m, U6, D6, U6_2, D6_2, false);
		//limitMechControl(&fourBar.m, U5, D5, U5_2, D5_2, FourBar.min, FourBar,nax, false);
		///limitMechControl(&lock.m, U8, D8, U8_2, D8_2, lock.min, lock.max, false);
		//binaryMechControl(&lock.m, U8, lock.max, lock.min, false, 50);
		analogMechControl(&baseRight.m, primary*vexRT[Ch2] + partner*vexRT[Ch2Xmtr2]);
		analogMechControl(&baseLeft.m, primary*vexRT[Ch3] + partner*vexRT[Ch3Xmtr2]);
		delay(10);
	}
}
/*******************************************************************************\
* .----------------.  .----------------.  .----------------.  .----------------. *
*| .--------------. || .--------------. || .--------------. || .--------------. |*
*| |   _____      | || |     _____    | || |  _________   | || |  _________   | |*
*| |  |_   _|     | || |    |_   _|   | || | |_   ___  |  | || | |  _   _  |  | |*
*| |    | |       | || |      | |     | || |   | |_  \_|  | || | |_/ | | \_|  | |*
*| |    | |   _   | || |      | |     | || |   |  _|      | || |     | |      | |*
*| |   _| |__/ |  | || |     _| |_    | || |  _| |_       | || |    _| |_     | |*
*| |  |________|  | || |    |_____|   | || | |_____|      | || |   |_____|    | |*
*| |              | || |              | || |              | || |              | |*
*| '--------------' || '--------------' || '--------------' || '--------------' |*
* '----------------'  '----------------'  '----------------'  '----------------' *
\*******************************************************************************/
void PIDLift(const struct liftMech* lift) {
	if (lift->PID.isRunning) mechMove(lift->m, pidController(lift->PID, lift->goal));//power the lift with its PID
	else resetPIDVals(lift->PID);//turn off the PID and reset values
	delay(lift->PIDelay);//delay a lil bit
}
void LiftLift(struct liftMech* lift, int bUp, int bDown, int bUp2, int bDown2, const float velLimit, const bool limited = true) {
	if (bUp || bDown || bUp2 || bDown2) {
		lift->PID.isRunning = false;
		if(limited)	limitMechControl(lift->m, bUp, bDown, bUp2, bDown2, lift->min, lift->max, false, 127);
		else			buttonMechControl(lift->m, bUp, bDown, bUp2, bDown2, false, 127);
	}
	else if (abs(SensorValue[lift->m.sensor] - lift->goal) < 200 || abs(lift->m.velocity) < velLimit) {
		if (!lift->PID.isRunning) lift->goal = SensorValue[lift->m.sensor];//sets goal if not already running
		lift->PID.isRunning = true;//now pid is definitely running
		PIDLift(lift);//calls the pid function for the lifts
	}
	else {
		lift->PID.isRunning = false;
		mechMove(lift->m, 0);
	}
	//PIDLift(lift);//calls the pid function for the lifts
}
void UpUntil(struct liftMech* lift, const int goal, const int speed) {
	lift->PID.isRunning = false;
	while (SensorValue[lift->m.sensor] < goal) {//brings lift up to goal
		mechMove(lift->m, speed);
	}
	lift->goal = SensorValue[lift->m.sensor];//keeps lift in last position
	lift->PID.isRunning = true;//re-enables pid
	return;
}
void DownUntil(struct liftMech* lift, const int goal, const int speed) {
	lift->PID.isRunning = false;
	while (SensorValue[lift->m.sensor] > goal) {//brings lift down to goal
		mechMove(lift->m, -speed);
	}
	lift->goal = SensorValue[lift->m.sensor];//keeps lift in last position
	lift->PID.isRunning = true;//re-enables pid
	return;
}
task intakeToLock(){
	while(SensorValue[MogoEnd] == 0){
		mechMove(&conveyer.m, INTAKE);
	}
	delay(200);
	lock.goal = lock.min;//turns on PID
	lock.PID.isRunning = true;
								/*while(SensorValue[MogoEnd] == 1){
									mechMove(&conveyer.m, INTAKE/6);
								}*/
	playSound(soundBeepBeep);
	//mechMove(&conveyer.m, 0);//stop conveyer
	LOCK;//brings up levitator
	lock.goal = SensorValue[lock.m.sensor];//turns on PID
	lock.PID.isRunning = true;
	//mechMove(&conveyer.m, 0);
	return;
}
task intakeToSecond(){
	//pick up second mogo
	while(SensorValue[MogoFront] == 0){
		mechMove(&conveyer.m, INTAKE);
	}
	delay(100);
	clawControl(OPEN);
	while(SensorValue[MogoFront] == 1){
		mechMove(&conveyer.m, INTAKE);//slower speed once intook (button pressed)
	}
	//clawControl(OPEN);clawControl(OPEN);clawControl(OPEN);clawControl(OPEN);//spams claw open just to make sure
	playSound(soundBlip);
	mechMove(&conveyer.m, 0);
	return;
}
task dropCone(){
	while(SensorValue[MogoFront] == 0)//wait until button pressed
		continue;
	playSound(soundUpwardTones);
	clawControl(OPEN);//spams claw open to ensure
	delay(1000);
	fourBar.goal = fourBar.min + 400;//then brings out the fourbar a bit
	fourBar.PID.isRunning = true;
}
task conveyerMove(){
	stopTask(MechControlTask);
	for(;;){
		if(autonRunning){
			mechMove(&conveyer.m, conveyer.speed);//allows multitasking
		}
		delay(20);
	}
}
task clawTask(){
	for(;;){
		clawControl(L8, R8);//allows manual claw control
		delay(100);
	}
}
task liftPID(){
	for(;;){
		if(autonRunning){//turns on PID for the lifts during auton
			PIDLift(&fourBar);
			PIDLift(&lock);
		}
		delay(20);
	}
}
/********************************************************************************\
* .----------------.  .----------------.  .----------------.  .----------------. *
*| .--------------. || .--------------. || .--------------. || .--------------. |*
*| |   ______     | || |  ____  ____  | || |  ____  ____  | || |    _______   | |*
*| |  |_   __ \   | || | |_   ||   _| | || | |_  _||_  _| | || |   /  ___  |  | |*
*| |    | |__) |  | || |   | |__| |   | || |   \ \  / /   | || |  |  (__ \_|  | |*
*| |    |  ___/   | || |   |  __  |   | || |    \ \/ /    | || |   '.___`-.   | |*
*| |   _| |_      | || |  _| |  | |_  | || |    _|  |_    | || |  |`\____) |  | |*
*| |  |_____|     | || | |____||____| | || |   |______|   | || |  |_______.'  | |*
*| |              | || |              | || |              | || |              | |*
*| '--------------' || '--------------' || '--------------' || '--------------' |*
* '----------------'  '----------------'  '----------------'  '----------------' *
\********************************************************************************/
float calcVel(struct mechanism* mech, float dist, float delayAmount) {
	float velocity = LimitDownTo(1, ((SensorValue(mech->sensor) - mech->past) / dist) / ((float)(delayAmount / 1000)));//1000 ms in 1s;
	mech->past = SensorValue[mech->sensor];
	return(velocity);//1000 ms in 1s;
}
float calcRotVel() {
	float velocity = mRot - pastRot;
	pastRot = mRot;
	return(velocity);
}
task MeasureSpeed() {
	/*MEASURING IN IN/SEC*/
	float dist = 1.125*PI;
	float delayAmount = 50;
	for (;;) {
		//base & conveyer velocities
		baseRight.m.velocity = calcVel(&baseRight.m, circum, delayAmount);
		baseLeft.m.velocity = calcVel(&baseLeft.m, circum, delayAmount);
		//overall bot velocities
		mainVelocity = avg(baseRight.m.velocity, baseLeft.m.velocity);
		rotVelocity = calcRotVel();
		//lift velocity
		lock.m.velocity = calcVel(&lock.m, dist, delayAmount);
		//does the waitings
		delay(delayAmount);
	}
}//task for measuring velocity of the base, in IN/Sec
/********************************************************************************\
* .----------------.  .----------------.  .----------------.  .----------------. *
*| .--------------. || .--------------. || .--------------. || .--------------. |*
*| |     ____     | || |  _________   | || |  ____  ____  | || |  _______     | |*
*| |   .'    `.   | || | |  _   _  |  | || | |_   ||   _| | || | |_   __ \    | |*
*| |  /  .--.  \  | || | |_/ | | \_|  | || |   | |__| |   | || |   | |__) |   | |*
*| |  | |    | |  | || |     | |      | || |   |  __  |   | || |   |  __ /    | |*
*| |  \  `--'  /  | || |    _| |_     | || |  _| |  | |_  | || |  _| |  \ \_  | |*
*| |   `.____.'   | || |   |_____|    | || | |____||____| | || | |____| |___| | |*
*| |              | || |              | || |              | || |              | |*
*| '--------------' || '--------------' || '--------------' || '--------------' |*
* '----------------'  '----------------'  '----------------'  '----------------' *
\********************************************************************************/
//for the cool ascii text go here: http://patorjk.com/software/taag/#p=display&f=Blocks
task sensorsUpdate() {
	//int rot=0;
	for (;;) {
		mRot = ((float)(GyroK*SensorValue[Gyro]));
		//encoderAvg = avg(SensorValue[baseRight.m.sensor], SensorValue[baseLeft.m.sensor]);

		delay(5);//really quick delay
	}
}
bool stalling(const struct mechanism* mech) {
	return (
	abs(motor[mech->motors[0]]) > 50 &&//high ish power
	abs(motor[mech->motors[1]]) > 50 &&//high ish power
	abs(mech->velocity) < 30//low ish velocity yet high speed (for like 500 ms)
	);
}
void checkStalling(struct mechanism* mech) {
	if (stalling(mech)) {
		clearTimer(T1);
		bool currentlyStalling = true;
		while (time1[T1] < 200) {//checkingn for continuous stalling (else instantanious refresh)
			currentlyStalling = stalling(mech);//still stalling
			if (currentlyStalling) continue;//keep going until time limit
			else break;
		}
		if (currentlyStalling)//done waiting, final check
			mech->stalling = true;//if so, consider it stalling
	}
	else mech->stalling = false;
	if (mech->stalling) playSound(soundBlip);
}

task antiStall() {
	for (;;) {
	//	checkStalling(&baseRight.m);
	//	checkStalling(&baseLeft.m);
		delay(50);
	}
}
task displayLCD(){
	for(;;){
		string mRotangle;
		//Display the Primary Robot battery voltage
		displayLCDString(0, 0, "Primary: ");
		sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
		displayNextLCDString(mainBattery);
		//Display the Power Expander voltage
		displayLCDString(1, 0, "Angle: ");
		sprintf(mRotangle, "%1.2f%c",   SensorValue[Gyro]*GyroK);//Build the value to be displayed
		displayNextLCDString(mRotangle);
		delay(30);
	}
}
/********************************************************************************\
* .----------------.  .----------------.  .----------------.  .-----------------.*
*| .--------------. || .--------------. || .--------------. || .--------------. |*
*| |      __      | || | _____  _____ | || |  _________   | || | ____  _____  | |*
*| |     /  \     | || ||_   _||_   _|| || | |  _   _  |  | || ||_   \|_   _| | |*
*| |    / /\ \    | || |  | |    | |  | || | |_/ | | \_|  | || |  |   \ | |   | |*
*| |   / ____ \   | || |  | |    | |  | || |     | |      | || |  | |\ \| |   | |*
*| | _/ /    \ \_ | || |   \ `--' /   | || |    _| |_     | || | _| |_\   |_  | |*
*| ||____|  |____|| || |    `.__.'    | || |   |_____|    | || ||_____|\____| | |*
*| |              | || |              | || |              | || |              | |*
*| '--------------' || '--------------' || '--------------' || '--------------' |*
* '----------------'  '----------------'  '----------------'  '----------------' *
\********************************************************************************/
void pre_auton() {//dont care
	bStopTasksBetweenModes = true;
	SensorType[Gyro] = sensorGyro;
	wait1Msec(1220);
	//Adjust SensorScale to correct the scaling for your gyro
	scaleGyros();
	while (bIfiRobotDisabled) {//in preauton
	//	autonSelect(nLCDButtons);
	}
}
task killswitch() {
	for (;;) {
		if (R7 && autonRunning) {
			stopAllTasks();
		}
		delay(50);
	}
}
void twentyScore(){
	UNLOCK;
	fwds(-127, mRot);
	delay(700);
	conveyer.speed = INTAKE;
	fwds(-127, mRot);
	delay(400);
	fwds(0, mRot);
	delay(400);
}
void crossField(const int initialDistance, const int secondaryDistance, bool withCone) {
	lock.goal = lock.max;
	//rot(100);
	//delay(5);
	UNLOCK;
	conveyer.speed = INTAKE/2;
	fourBar.goal = RELEASE + 500;
	fourBar.PID.isRunning = true;
	startTask(intakeToLock);//picks up first mogo
	driveFor(initialDistance);
	fourBar.goal = RELEASE;
	fwds(70, mRot);
	delay(500);
	startTask(intakeToSecond);
	if (withCone) startTask(dropCone);
	driveFor(secondaryDistance);
	if (withCone) startTask(dropCone);
	fourBar.goal = avg(fourBar.min, fourBar.max);
	conveyer.speed = INTAKE / 6;
	delay(300);
	lock.goal = lock.min;
	settle();
}
volatile int clawSpeed = 0;
task clawHold(){
	for(;;){
		motor[ClawMotor] = clawSpeed;
		delay(10);
	}
}
void timeEmpty(int target){
	rot(getSign(target)*127);
	if(abs(target) == 90) delay(420);
	else if(abs(target) == 45) delay(270);
	else delay(400);
	settle();
}

void timeDual(int target){
	rot(getSign(target)*127);
	if(abs(target) == 90) delay(630);
	else delay(500);
	settle();
}
void progSkillsTest(){
	startTask(conveyerMove);
	startTask(killswitch);
	startTask(liftPID);
	autonRunning = true;
	//resetGyros();


																					//cross field pickup 2 mogos
																						crossField(80, 70, true);
																					//crossField(63,74,true);
																						//alignToLine(1);
																					//align to 20 and score
																						//driveFor(6);
																					conveyer.speed = 0;
																					delay(100);
																					rotAcc(87.5, 1000);
																					fourBar.goal = RELEASE;
																					delay(300);
																					driveFor(-12.5);//-12);
																					settle();
																					delay(100);
																					rotAcc(85);
																					//rot(127);//TIME BASED TURN
																					//delay(500);
																					//rot(0);
																					delay(50);
																					SensorValue[Gyro] = 0;
																					fourBar.goal = avg(fourBar.min, fourBar.max);
																					lock.PID.isRunning = false;
																					conveyer.speed = 0;
																					twentyScore();//SCORES FIRST TWO IN ZONE
																					settle();
																					delay(300);
																					//untilLine(127);
																					driveFor(30);
																					fourBar.goal = RELEASE;//brings to rear again
																					//'s' motion
																					alignToLine(1);//aligns to tape line
																					conveyer.speed = INTAKE/8;//turn off conveyer
																					settle();
																					//fourBar.goal = PICKUP;
																					delay(200);
																					rotAcc(90);
																					settle();
																					alignSonar(42);
																					settle();
																					delay(400);
																					rotAcc(-88);
																					settle();
																					//alignToLine(-1);//aligns to tape line
																					delay(300);
																					//cross field pickup 2 mogos
																					crossField(80, 62, false);
																					fourBar.goal = RELEASE;
																					conveyer.speed = 0;
																					rotAcc(90);
																					driveFor(-11);
																					settle();
																					rotAcc(85);
																					lock.PID.isRunning = false;
																					twentyScore();
																					driveFor(26);
	alignToLine(1);
	rotAcc(90);
	alignSonar(27);
	settle();
	delay(200);
	rotAcc(-46,1000);
	int initialMRot = SensorValue[Gyro]*GyroK;
	//rot(-127);
	//delay(270);//TIME BASED 45er
	//SensorValue[LeftBaseEnc] = 0;
	//while(abs(SensorValue[LeftBaseEnc]) < 100){continue;}
	settle();
	delay(50);
//	alignToLine(0.9);
	startTask(intakeToLock);
	delay(200);
	driveFor(80);
	clearTimer(T1);
	while(abs(SensorValue[Gyro]*GyroK - initialMRot) > 1 && time1[T1] < 500)	rot(-10*(SensorValue[Gyro]*GyroK - initialMRot));
	settle();
	delay(300);
	driveFor(-7);
	fourBar.goal = PICKUP;
	clawSpeed = -127;
	startTask(clawHold);
	delay(400);
	rotAcc(-18.5, 800);
	settle();
	delay(100);
	driveFor(9);
	clawSpeed = 100;//pickup cone
	delay(400);
	stopTask(clawHold);
	delay(400);//settling
	DownUntil(&fourBar, fourBar.min, 80);
	delay(100);
	settle();
	delay(300);
	rotAcc(-11, 300);
	startTask(dropCone);
	startTask(intakeToSecond);
	driveFor(34);
	//clawControl(OPEN);
	delay(600);
	fourBar.goal = PICKUP;
	conveyer.speed = 0;
	delay(500);
	rotAcc(-50, 1000);
	//DONE
	autonRunning = false;
	startTask(MechControlTask);
	return;
}
task autonomous() {
	autonRunning = true;
	initializeOpControl(false);//auton init
	startTask(MechControlTask);//individual pid for lift type
	startTask(MeasureSpeed);//velocity measurer for base
	startTask(sensorsUpdate);
	startTask(conveyerMove);//allows for multitasking
	//startTask(antiStall);
	startTask(clawTask);
	progSkillsTest();

	return;
}
/********************************************************************************\
* .----------------.  .----------------.  .----------------.  .----------------. *
*| .--------------. || .--------------. || .--------------. || .--------------. |*
*| | _____  _____ | || |    _______   | || |  _________   | || |  _______     | |*
*| ||_   _||_   _|| || |   /  ___  |  | || | |_   ___  |  | || | |_   __ \    | |*
*| |  | |    | |  | || |  |  (__ \_|  | || |   | |_  \_|  | || |   | |__) |   | |*
*| |  | '    ' |  | || |   '.___`-.   | || |   |  _|  _   | || |   |  __ /    | |*
*| |   \ `--' /   | || |  |`\____) |  | || |  _| |___/ |  | || |  _| |  \ \_  | |*
*| |    `.__.'    | || |  |_______.'  | || | |_________|  | || | |____| |___| | |*
*| |              | || |              | || |              | || |              | |*
*| '--------------' || '--------------' || '--------------' || '--------------' |*
* '----------------'  '----------------'  '----------------'  '----------------' *
\********************************************************************************/
task usercontrol() {//initializes everything
	initializeOpControl(true);//driver init
	startTask(MechControlTask);//individual pid for lift type
	startTask(MeasureSpeed);//velocity measurer for base
	startTask(sensorsUpdate);
	startTask(antiStall);
	startTask(killswitch);
	startTask(displayLCD);
	startTask(clawTask);
	SensorValue[Gyro] = 0;//resets gyros
	SensorScale[Gyro] = 260;
	autonRunning = false;
	fourBar.goal = RELEASE;
	if(nImmediateBatteryLevel < 8300) playSound(soundException);
	else playSound(soundUpwardTones);
	for (;;) {
		//debug controls
		//if(U8) progSkillsTest();
		if(L7){// timeempty(45); //alignToLine();
			rotAcc(45);
		}
		if(R7) rotAcc(-45);//clawControl(OPEN);
		if(U7) progSkillsTest();
		if(D7) turn(45, 2);
		LiftLift(&fourBar, D5, U5, D5_2, U5_2, 5000, true);
		LiftLift(&lock, U8, D8, U8_2, D8_2, 100, true);
	//	LOCK;
	//	delay(300);
	//	UNLOCK;
		delay(30);//~60hz
	}
}//function for operator control

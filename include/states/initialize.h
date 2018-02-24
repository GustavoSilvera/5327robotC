#if !defined(INIT_H)
#define INIT_H

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
struct PIDs {
	char sensor;
	volatile int forceDirection;//changing with multiple threads
	int motor1, motor2, thresh;
	float kP, kI, kD;
	bool reversed, Mopposite;
	volatile bool isRunning;
	float Integral, Derivative, LastError;
};
struct PIDs gyroBase;

enum liftType{BINARY, DIFFERENTIAL, NORMAL, NOPID};//what kinds of lift we have

struct liftMech {
	enum liftType type;
	int motors[2];
	char sensor;
	float max, min, liftPIDelay, past;
	struct PIDs PID;
	volatile float goal;
	float velocity;
};

struct liftMech mainLift;
struct liftMech mogo;
struct liftMech FourBar;

struct sideBase{
	int motors[3];
	char sensor;
	float velocity;
	volatile bool stalling;
	float past;
};
struct sideBase Left;
struct sideBase Right;
struct sideBase goliat;

void initLiftType(const struct liftMech* lift, enum liftType type, char sensor, int m1, int m2, int max, int min, int delayAmnt = 20) {
	lift->type = type;
	lift->sensor = sensor;
	lift->motors[0] = m1;
	lift->motors[1] = m2;
	lift->max = max;
	lift->min = min;
	lift->liftPIDelay = delayAmnt;
	lift->velocity = 0.0;
	lift->past = 0;
	lift->goal = SensorValue[lift->sensor];
}
void initPID(const struct PIDs* PIDType, char sensor, int thresh, float kP, float kI, float kD, bool reversed, bool isRunning = true) {
	PIDType->sensor = sensor;
	PIDType->thresh = thresh;
	PIDType->kP = kP;//0.2 //pretty efficient lift tho 0.075
	PIDType->kI = kI;// 0 .05;//0.04;
	PIDType->kD = kD;//1;
	PIDType->LastError = 0.0;
	PIDType->Integral = 0.0;
	PIDType->reversed = reversed;
	PIDType->isRunning = isRunning;
}
void initSideBase(const struct sideBase* side, char sensor, int m1, int m2, int m3){
	side->sensor = sensor;
	side->motors[0] = m1;
	side->motors[1] = m2;
	side->motors[2] = m3;
	side->velocity = 0;//initially not moving
	side->stalling = false;//initially not stalling (i hope)
	SensorValue[side->sensor] = 0;//initially resets encoders
	side->past = 0;
}
#endif

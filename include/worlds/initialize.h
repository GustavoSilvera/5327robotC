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
	int thresh, goal, refresh;
	float kP, kI, kD;
	bool isReversed, isRunning;
	float Integral, Derivative, Last;
};
struct PIDs gyroBase;
enum liftType{
	BINARY,
	DIFFERENTIAL,
	NORMAL,
	NOSENSOR,
	NOPID
};//what kinds of lift we have
struct liftMech {
	enum liftType type;
	int motors[2], max, min, past;
	char sensor;
	struct PIDs PID;
	float velocity;
	bool isReversed, isStalling;
};
struct liftMech mainLift;
struct liftMech MoGo;
struct liftMech FourBar;
struct liftMech goliat;

struct sideBase{
	int motors[3];
	char sensor;
	float velocity, past;
	bool isStalling;
};
struct sideBase Left;
struct sideBase Right;
//struct sideBase goliat;

void initLiftType(const struct liftMech* lift, enum liftType type, char sensor, int m1, int m2, int max, int min, bool reversed = false) {
	lift->type = type;
	lift->sensor = sensor;
	lift->motors[0] = m1;
	lift->motors[1] = m2;
	lift->max = max;
	lift->min = min;
	lift->velocity = 0.0;
	lift->past = 0;
	lift->isReversed = reversed;
	lift->isStalling = false;
}
void initPID(const struct PIDs* PIDType, char sensor, int thresh, float kP, float kI, float kD, bool reversed, bool isRunning = true) {
	PIDType->thresh = thresh;
	PIDType->kP = kP;//0.2 //pretty efficient lift tho 0.075
	PIDType->kI = kI;// 0 .05;//0.04;
	PIDType->kD = kD;//1;
	PIDType->Last= 0.0;
	PIDType->Integral = 0.0;
	PIDType->isReversed = reversed;
	PIDType->isRunning = isRunning;
	PIDType->goal = 0;
	PIDType->refresh = 20;//ms for pid delay
}
void initSideBase(const struct sideBase* side, char sensor, int m1, int m2, int m3){
	side->sensor = sensor;
	side->motors[0] = m1;
	side->motors[1] = m2;
	side->motors[2] = m3;
	side->velocity = 0;//initially not moving
	side->isStalling = false;//initially not stalling (i hope)
	SensorValue[side->sensor] = 0;//initially resets encoders
	side->past = 0;
}
#endif

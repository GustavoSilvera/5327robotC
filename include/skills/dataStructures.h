int goalTicks;
int goalPower;
int encoderAvg;

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
enum mechType { CONVEY, DRIVE };//what kinds of lift we have
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
static const float GyroK = 15.0/80.0;//scales to normal +-360 degrees
volatile bool autonRunning = false;
string mainBattery, powerExpander;
//int startRot = 90;
volatile float mRot = 0.0;//current rotation
//volatile float encoderAvg;//used only for straight fwds and bkwds direction

//MISC FUNCTIONS
//use macros instead :)
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
//float sqr(float val) {
//	return val*val;
//}
float avg(float a, float b) {
	return 0.5*(a + b);//avg between two things
	// a + b >>2;
}

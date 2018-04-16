#if !defined(RANDOM_H)
#define RANDOM_H

/*
 .----------------.  .----------------.  .-----------------. .----------------.  .----------------.  .----------------.
| .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
| |  _______     | || |      __      | || | ____  _____  | || |  ________    | || |     ____     | || | ____    ____ | |
| | |_   __ \    | || |     /  \     | || ||_   \|_   _| | || | |_   ___ `.  | || |   .'    `.   | || ||_   \  /   _|| |
| |   | |__) |   | || |    / /\ \    | || |  |   \ | |   | || |   | |   `. \ | || |  /  .--.  \  | || |  |   \/   |  | |
| |   |  __ /    | || |   / ____ \   | || |  | |\ \| |   | || |   | |    | | | || |  | |    | |  | || |  | |\  /| |  | |
| |  _| |  \ \_  | || | _/ /    \ \_ | || | _| |_\   |_  | || |  _| |___.' / | || |  \  `--'  /  | || | _| |_\/_| |_ | |
| | |____| |___| | || ||____|  |____|| || ||_____|\____| | || | |________.'  | || |   `.____.'   | || ||_____||_____|| |
| |              | || |              | || |              | || |              | || |              | || |              | |
| '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
 '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------'
*/
#define PI 3.1415
//controller
#define U5    		vexRT[Btn5U]//5U
#define D5	  		vexRT[Btn5D]//5D
#define U6  		vexRT[Btn6U]//6U
#define D6	  		vexRT[Btn6D]//6D
#define U7  		vexRT[Btn7U]//7U
#define D7	  		vexRT[Btn7D]//7D
#define L7    		vexRT[Btn7L]//7L
#define R7			vexRT[Btn7R]//7R
#define U8  		vexRT[Btn8U]//8U
#define D8	  		vexRT[Btn8D]//8D
#define L8    		vexRT[Btn8L]//8L
#define R8	  		vexRT[Btn8R]//8R
//partner controller
#define U5_2    	vexRT[Btn5UXmtr2]//5U2
#define D5_2	    vexRT[Btn5DXmtr2]//5D2
#define U6_2		vexRT[Btn6UXmtr2]//6U2
#define D6_2	    vexRT[Btn6DXmtr2]//622
#define U7_2  		vexRT[Btn7UXmtr2]//7U2
#define D7_2	    vexRT[Btn7DXmtr2]//7D2
#define L7_2    	vexRT[Btn7LXmtr2]//7L2
#define R7_2	    vexRT[Btn7RXmtr2]//7R2
#define U8_2  		vexRT[Btn8UXmtr2]//8U2
#define D8_2	    vexRT[Btn8DXmtr2]//8D2
#define L8_2    	vexRT[Btn8LXmtr2]//8L2
#define R8_2	    vexRT[Btn8RXmtr2]//8R2

#define LEFTside    false
#define RIGHTside   true
#define TEN         false
#define TWENTY      true
#define OUTTAKE

//other
const float circum = 12.56;//4*PI;//4 inch wheels
float rotVelocity, pastRot;
static const float GyroK = 0.1875;//(15.0/80.0);//scales to normal +-360 degrees
int initDir = SensorValue[Gyro] * GyroK;
static int currentAutonomous = 0, currentCone = 0;
const bool slewRating = false;
bool autoStacking = false, autonRunning = false, matchLoads = false;
string mainBattery, powerExpander, currCone, gyroRead;
volatile float encoderAvg = 0, velocity = 0;//used only for straight fwds and bkwds direction
int holdPower = 0;//for goliath hold
bool hasCone = false;
int intakeSpeed = 0;
//MISC FUNCTIONS
//use macros!!! :)
float limitUpTo(const float max, float val) {
	if (abs(val) < abs(max)) return val;
	else return sgn(val) * max;
}
float limitDownTo(const float min, const float val) {
	if (abs(val) > abs(min)) return val;
	else return sgn(val) * min;
}
float avg2(const float a, const float b){
	return 0.5*(a+b);//avg between two things
}
#endif

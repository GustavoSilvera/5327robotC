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
const float circum = 4 * PI;//4 inch wheels
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

#define LEFT false
#define RIGHT true
#define TEN false
#define TWENTY true

/* YOU SHALL NOT PASS!!!! BEGONE!
#define SQR(x) (x*x)
#define CUB(x) (x*x*x)
#define QUAR(x) (x*x*x*x)
#define CINQ(x) (x*x*x*x*x)
#define AVG(x,y) ((x+y)/2)
#define AVGINT(x,y) ((x+y)>1)
#define GETSIGN(x) (x < 0 ? -1 : 1)
//#define LIMITUP(max, val) (val < max ? val : max)
#define LIMITUP(max, val) ((abs(val)) < abs(max) ? val : (max * GETSIGN(val)))
//#define LIMITDOWN(min, val) (val > min ? val : min)
#define LIMITDOWN(min, val) ((abs(val)) > abs(min) ? val : (min * GETSIGN(val)))
*/
//other
static volatile float velocity = 0;
float rotVelocity = 0;
float pastRot;
int currentCone = 0;
int currentStag = 0;
static const float GyroK = 15.0/80.0;//scales to normal +-360 degrees
static int currentAutonomous = 0;
bool autonRunning = false;
volatile bool autoStacking = false;
string mainBattery, powerExpander, currCone, gyroRead;
//int startRot = 90;
volatile float mRot;//current rotation
volatile float encoderAvg;//used only for straight fwds and bkwds direction
//MISC FUNCTIONS
//use macros!!! :)...ew gross
int getSign(const float val){
	if(val < 0) return -1;
	else if(val > 0) return 1;
	else return 0;
}
float limitUpTo(const float max, float val) {
	if (abs(val) < abs(max)) return val;
	else return getSign(val) * max;
}
float limitDownTo(const float min, const float val) {
	if (abs(val) > abs(min)) return val;
	else return getSign(val) * min;
}
float avg2(const float a, const float b){
	return 0.5*(a+b);//avg between two things
}
#endif

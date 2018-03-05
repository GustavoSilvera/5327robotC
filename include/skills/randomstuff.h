#if !defined(RANDOM_H)
#define RANDOM_H
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


	//other stuff and #defines
	#define PI 3.1415926535

	#define LEFT true
	#define RIGHT false

	#define INT_MATH_SHIFT 16
	#define SQUARE(x) (x * x)
	#define AVG(x,y) ((x + y)/2)
	#define AVGINT(x,y) ((x + y)>>1)
	#define MIN(x,y) (x <= y ? x : y)
//	#define GETSIGN(x) (x < 0 ? -1 : 1)
	#define LIMITUP(max, val) (val < max ? val : max)
	#define LIMITDOWN(min, val) (val > min ? val : min)

	#define INTAKE 127
	#define OUTTAKE -127
	#define OPEN 1, 0
	#define CLOSE 0, 1
	#define RELEASE fourBar.min
	#define PICKUP 3 fourBar.max
	#define UNLOCK 	UpUntil(&lock, lock.max, 127)
	#define LOCK DownUntil(&lock, lock.min, 127)
	const float circum = 4 * PI;//4 inch wheels



#endif

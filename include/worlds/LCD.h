#if !defined(LCD_H)
#define LCD_H
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
/*for the cool ascii text go here: http://patorjk.com/software/taag/#p=display&f=Blocks*/
void displayAuton( int value, bool select, bool coneSelector ){
	// Cleat the lcd
	clearLCDLine(0);
	clearLCDLine(1);

	if(coneSelector){
		displayLCDString(1, 5, "ACTIVE");
		// Display the selection operations
		displayLCDString(1, 0, "-1 ");
		displayLCDString(1, 13, " +1");
		// Simple selection display
		switch (value) {
		case    0:
			autonConeNum = 0;
			break;
		case    1:
			autonConeNum = 1;
			break;
		case    2:
			autonConeNum = 2;
			break;
		default:
			autonConeNum = 3;
			break;
		}
	}
	else {//in auton choice mode
		if (select) autonIndex = value;//updates current auton with new
		if (autonIndex == value) displayLCDString(1, 5, "ACTIVE");
		else displayLCDString(1, 5, "select");
		// Display the selection arrows
		displayLCDString(1, 0, "<--");
		displayLCDString(1, 13, "-->");

		// Simple selection display
		switch (value) {
		case    0:
			customAuton = "20 R";
			autonConeNum = 0;
			break;
		case    1:
			customAuton = "20 L";
			autonConeNum = 0;
			break;
		case    2:
			customAuton = "10 R";
			autonConeNum = 0;
			break;
		case    3:
			customAuton = "10 L";
			autonConeNum = 0;
			break;
		case    4:
			customAuton = "LODR";//loader right
			autonConeNum = 0;
			break;
		case    5:
			customAuton = "LODL";//loader left
			autonConeNum = 0;
			break;
		case    6:
			displayLCDString(0, 0, " NONE ");
			break;
		default:
			displayLCDString(0, 0, " NONE ");
			break;
		}
	}
	string coneNumString ;
	sprintf(coneNumString, " +%d", autonConeNum + 1); //Build the value to be displayed
	strcat(customAuton, coneNumString);//concatenates the two strings together
	displayLCDString(0, 0, customAuton);

}
void autonSelect(int delayTime = 5000){//5 second wait time
	clearTimer(T4);
	int value = 0;//no auton
	// here for reference http://help.robotc.net/Sandbox/Zendesk-Output/Content/Resources/topics/VEX_Cortex/ROBOTC/LCD_Display/nLCDButtons.htm
	const int LEFT = 1;
	const int RIGHT = 4;
	const int CENTER = 2;
	intrinsic const static volatile unsigned int NUMAUTONS = 7;
	autonConeNum = 0;//restarts cone counter
	bool selectingCone = false;
	while(time1[T4] < delayTime){
		// display default choice
		displayAuton(value, false, selectingCone);
		// Display and select the autonomous routine
		if( ( nLCDButtons == LEFT ) || ( nLCDButtons == RIGHT) ) {
			// previous choice
			if( nLCDButtons == LEFT && value > 0)
				value--;
			// next choice
			if( nLCDButtons == RIGHT && value < NUMAUTONS)
				value ++;
			displayAuton(value, false, selectingCone);//dosent say "ACTIVE"
			clearTimer(T4);
		}
		// Select this choice
		if( nLCDButtons == CENTER) {
			selectingCone = true;
			displayAuton(value, true, selectingCone);//says "ACTIVE" & with cone selector
			value = 0;
			clearTimer(T4);
		}
		delay(200);
	}
}
void displayStuff(){
	//Display the Primary Robot battery voltage
	displayLCDString(0, 0, "B:");
	sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
	displayNextLCDString(mainBattery);
	displayNextLCDString(" ");
	//Display the Power Expander voltage
	displayNextLCDString("Ex:");
	sprintf(powerExpander, "%1.2f%c", ((float)SensorValue[ BATERY_2_PORT ] * 5.48/1000), 'V');//Build the value to be displayed
	displayNextLCDString(powerExpander);
	//Display the CurrentCone
	displayLCDString(1, 0, "Cc:");
	sprintf(currCone, "%d", (int)currentCone, 'c');//Build the value to be displayed
	displayNextLCDString(currCone);
	displayNextLCDString(" ");
	//Display the Power Expander voltage
	displayNextLCDString("Gy:");
	sprintf(gyroRead, "%1.2f%c", SensorValue[Gyro] * GyroK);//Build the value to be displayed
	displayNextLCDString(gyroRead);
}
task displayLCD(){
	for(;;){
		if(nLCDButtons != 0) { //not sure if works (displays for 5 sec)
			autonSelect();
		}//displays auton for 5 seconds
		else displayStuff();
		delay(50);
	}
}
#endif

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
void displayAuton( int value, bool select = false  ){
	// Cleat the lcd
	clearLCDLine(0);
	clearLCDLine(1);

	// Display the selection arrows
	displayLCDString(1,  0, "<--");
	displayLCDString(1, 13, "-->");
	// Save autonomous mode for later if selected
	if(select) currentAutonomous = value;//updates current auton with new
		// If this choice is selected then display ACTIVE
	if( currentAutonomous == value )
		displayLCDString(1, 5, "ACTIVE");
	else
		displayLCDString(1, 5, "select");

	// Simple selection display
	switch(value){
	case    0:
		displayLCDString(0, 0, "3 Cone R");
		break;
	case    1:
		displayLCDString(0, 0, "3 Cone L");
		break;
	case    2:
		displayLCDString(0, 0, "1 Cone R");
		break;
	case    3:
		displayLCDString(0, 0, "1 Cone L");
		break;
	case    4:
		displayLCDString(0, 0, "RAM time");
		break;
	case    5:
		displayLCDString(0, 0, "no auton");
		break;
	default:
		displayLCDString(0, 0, "Unknown");
		break;
	}

	// Save autonomous mode for later
	//currentAutonomous = value;
}
void autonSelect(int delayTime = 5000){
	clearTimer(T4);
	int value = 0;//no auton
	// here for reference http://help.robotc.net/Sandbox/Zendesk-Output/Content/Resources/topics/VEX_Cortex/ROBOTC/LCD_Display/nLCDButtons.htm
	const int LEFT = 1;
	const int RIGHT = 4;
	const int CENTER = 2;
	while(time1[T4] < delayTime){
		// diaplay default choice
		displayAuton(value);
		// Display and select the autonomous routine
		if( ( nLCDButtons == LEFT ) || ( nLCDButtons == RIGHT) ) {
			// previous choice
			if( nLCDButtons == LEFT && value > 0)
				value--;
			// next choice
			if( nLCDButtons == RIGHT && value < 5)
				value ++;
			displayAuton(value);//dosent say "ACTIVE"
			clearTimer(T4);
		}
		// Select this choice
		if( nLCDButtons == CENTER ) {
			displayAuton(value, true );//says "ACTIVE"
			clearTimer(T4);
		}
		delay(200);
	}
}
void displayBatteryLevels(){
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
	sprintf(gyroRead, "%1.2f%c", SensorValue[Gyro], "°");//Build the value to be displayed
	displayNextLCDString(gyroRead);


}
task displayLCD(){
	for(;;){
		if(nLCDButtons != 0) { //not sure if works (displays for 5 sec)
			autonSelect();
		}//displays auton for 5 seconds
		else displayBatteryLevels();
		delay(30);
	}
}
#endif

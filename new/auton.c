#include "main.c"
const int enterString[] = {247,32,32,32,32,32,69,110,116,101,114,32,32,32,32,246};//this is "�??     Enter    �?�"
const string firstAutonString = "Left Side MGM";
const string secondAutonString = "Right Side MGM";
const string thirdAutonString = "Stationary";
const string fourthAutonString = "Nothing";
int lcdCount = 0;
const int C_rOfWheels = 2;
const int C_rOfRobot = 13;
const float C_PI = 3.1415926;
const int C_motorPower = 100;
const float C_dr4bconstant = 2.5;
void displayEnterString(int line){//this displays *enterString[]* to *line*
	clearLCDLine(line);
	for(int i = 0; i < 16; ++i){
		displayLCDChar(line, i, enterString[i]);
	}
}
int getEncValForDistance(int inches){//this returns the encoder value for drivetrain distance
	return (360*inches)/(C_rOfWheels*C_PI*2);
}
int getEncValForTurn(int degrees){//this returns how many times an encoder on the drivetrain needs to turn in relation to how far the robot needs to turn
	return degrees*C_rOfRobot/C_rOfWheels;
}
void moveForwards(int distance){//this moves the robot forwards *distance* inches
	int encVal = getEncValForDistance(distance);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	motor[ldt1] = motor[ldt2] = C_motorPower;
	motor[rdt1] = motor[rdt2] = -C_motorPower;
	while(encVal<SensorValue[ldtEnc]) wait1Msec(5);
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void moveBackwards(int distance){//this moves the robot backwards *distance* inches
	int encVal = getEncValForDistance(distance);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	motor[ldt1] = motor[ldt2] = -C_motorPower;
	motor[rdt1] = motor[rdt2] = C_motorPower;
	while(encVal>SensorValue[ldtEnc]) wait1Msec(5);
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void turnRight(int degrees){//this turns the robot to the right *degrees* degrees
	//gyroTurn((float)degrees);
	motor[ldt1] = motor[ldt2] = C_motorPower;
	motor[rdt1] = motor[rdt2] = C_motorPower;
	wait1Msec((15*1000)/(7*degrees));
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void turnLeft(int degrees){//this turns the robot to the left *degrees* degrees
	//gyroTurn((float)(-degrees));
	motor[ldt1] = motor[ldt2] = -C_motorPower;
	motor[rdt1] = motor[rdt2] = -C_motorPower;
	wait1Msec((15*1000)/(7*degrees));
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void lowerMGM(){//lowers mgm
	motor[mgm]= C_motorPower;
	wait1Msec(500);
	motor[mgm] = 0;
}
void raiseMGM(){//raises mgm
	motor[mgm] = -C_motorPower;
	wait1Msec(500);
	motor[mgm] = 0;
}
void rotateDr4bUpTo(float distance){//rotates double reverse fourbar up to *distance* height
	int encVal = abs(distance*C_dr4bconstant);
	SensorValue[rdr4bEnc]=0;
	SensorValue[ldr4bEnc]=0;
	while(SensorValue[ldr4bEnc]>-encVal && SensorValue[rdr4bEnc]<encVal){
		motor[ldr4b] = -C_motorPower;
		motor[rdr4b] = C_motorPower;
		encVal = abs(distance*C_dr4bconstant);
	}
	motor[ldr4b] = motor[rdr4b] = 0;
}
void rotateDr4bDownTo(float distance){//rotates double reverse fourbar down to *distance* height
	SensorValue[rdr4bEnc]=0;
	SensorValue[ldr4bEnc]=0;

	int encVal = abs(distance*C_dr4bconstant);
	while(SensorValue[ldr4bEnc]<encVal && SensorValue[rdr4bEnc]>-encVal){
		motor[ldr4b] = C_motorPower;
		motor[rdr4b] = -C_motorPower;
		encVal = abs(distance*C_dr4bconstant);
	}
	motor[ldr4b] = motor[rdr4b] = 0;
}
void harvesterUp(){
	motor[claw] = C_motorPower;
	wait1Msec(500);
	motor[claw] = 0;
}
void harvesterDown(){
	motor[claw] = -C_motorPower;
	wait1Msec(1000);
	motor[claw] = 0;
}
void rotateFourbarTo(int degrees){

}
void robotInit(){
	motor[claw]=75;
	rotateFourbarTo(33);
}
void returnToLowered(){
	motor[ldr4b] = C_motorPower;
	motor[rdr4b] = -C_motorPower;
	wait1Msec(1000);
	motor[ldr4b] = motor[rdr4b] = 0;
	wait1Msec(200);
	motor[fourbar] = -C_motorPower;
	wait1Msec(1000);
	motor[fourbar] = 0;
	SensorValue[rdr4bEnc]=0;
	SensorValue[ldr4bEnc]=0;
	SensorValue[ldtEnc]=0;
	SensorValue[rdtEnc]=0;
}
task auton(){//main task
	getEncValForTurn(1);
	switch(lcdCount){
		case 0://first auton
			displayLCDString(0,0, firstAutonString);
			displayLCDString(1,0, "is running!");
			robotInit();
			moveForwards(40);
			harvesterUp();
			rotateDr4bUpTo(30);
			lowerMGM();
			moveForwards(8);
			harvesterDown();
			raiseMGM();
			moveBackwards(8);
			turnRight(180);
			moveForwards(36);
			turnLeft(90);
			moveForwards(18);
			turnRight(90);
			moveForwards(16);
			turnRight(45);
			moveForwards(36);
			lowerMGM();
			moveBackwards(8);
			raiseMGM();
			moveBackwards(8);
			break;
		case 1:
			displayLCDString(0,0, secondAutonString);
			displayLCDString(1,0, "is running!");
			robotInit();
			moveForwards(40);
			harvesterUp();
			rotateDr4bUpTo(30);
			lowerMGM();
			moveForwards(8);
			harvesterDown();
			raiseMGM();
			moveBackwards(8);
			turnLeft(180);
			moveForwards(36);
			turnRight(90);
			moveForwards(18);
			turnLeft(90);
			moveForwards(16);
			turnLeft(45);
			moveForwards(36);
			lowerMGM();
			moveBackwards(8);
			raiseMGM();
			moveBackwards(8);
			break;
		case 2:
			displayLCDString(0,0, thirdAutonString);
			displayLCDString(1,0, "is running!");
			robotInit();
			rotateDr4bUpTo(35);
			rotateFourbarTo(75);
			moveForwards(3);
			rotateFourbarTo(50);
			wait1Msec(500);
			moveForwards(2.5);
			wait1Msec(1000);
			motor[ldr4b] = motor[rdr4b] = 0;
			motor[claw] = -100;
			wait1Msec(2000);
			motor[claw] = 0;
			moveBackwards(7);
			returnToLowered();
			break;
		case 3:
			displayLCDString(0,0, fourthAutonString);
			displayLCDString(1,0, "is running!");
			break;
		default:
			break;
	}
}

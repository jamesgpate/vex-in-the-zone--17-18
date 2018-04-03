#include "main.c"
#include "driving.c"
#include "lights.c"
#include "gyroLib.c"
float gyroCorrectedAngle = 0.0;
const int enterString[] = {247,32,32,32,32,32,69,110,116,101,114,32,32,32,32,246};//this is "�??     Enter    �?�"

const string zerothAutonString = "Stationary Goal"; //Verified

const string firstAutonString = "redLeft 3 20"; //Verified
const string secondAutonString = "redLeft 2 20";
const string thirdAutonString = "redLeft 1 20";
const string fourthAutonString = "redLeft 3 10";

const string fifthAutonString = "redRight 3 20";
const string sixthAutonString = "redRight 2 20";
const string seventhAutonString = "redRight 1 20";
const string eighthAutonString = "redRight 3 10";

const string ninthAutonString = "blueRight 3 20";
const string tenthAutonString = "blueRight 2 20";
const string eleventhAutonString = "blueRight 1 20";
const string twelfthAutonString = "blueRight 3 10";

const string thirteenthAutonString = "blueLeft 3 20";
const string fourteenthAutonString = "blueLeft 2 20";
const string fifteenthAutonString = "blueLeft 1 20";
const string sixteenthAutonString = "blueLeft 3 10";

const string seventeenthAutonString = "Nothing"; //8
const string eighteenthAutonString = "Gyro Turn Test";
int lcdCount = 0;

const int C_rOfWheels = 2;
const int C_rOfRobot = 13;
const float C_PI = 3.1415926;
const int C_motorPower = 120;
const float C_dr4bconstant = 2.5;
//for left encoder, forwards is pos, backwards is neg
//for right encoder, forwards is neg, backwards is pos

void displayEnterString(int line){//this displays *enterString[]* to *line*
	clearLCDLine(line);
	displayLCDString(line, 0,"<     Enter    >");
}

int getEncValForDistance(int inches){//this returns the encoder value for drivetrain distance
	return (360*inches)/(C_rOfWheels*C_PI*2);
}
int getEncValForTurn(int degrees){//this returns how many times an encoder on the drivetrain needs to turn in relation to how far the robot needs to turn
	return degrees*((float)C_rOfRobot/(float)C_rOfWheels);
}
void moveForwards(int distance){//this moves the robot forwards *distance* inches
	int encVal = getEncValForDistance(distance);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	while(!(SensorValue[rdtEnc]<(-encVal+10) && SensorValue[rdtEnc]>(-encVal-10))){
		motor[ldt1]=motor[ldt2]=90;
		motor[rdt1]=motor[rdt2]=-90;

	}
	motor[ldt1]=motor[ldt2]=0;
	motor[rdt1]=motor[rdt2]=0;
}
float gyroScale(float gyroScale){
	gyroCorrectedAngle = GyroAngleAbsGet()*gyroScale;
	return gyroCorrectedAngle;
}
void gyroTurn(float degrees, int power){
	int error, direction;
	float gyroScaleFloat = sqrt(2);
	wait1Msec(200);
	while(!GyroValidGet()) wait1Msec(5);
	if(degrees < 0) direction = 1;
	if(degrees > 0) direction = 0;
	if(direction == 1){//if turn is clockwise
		while(gyroScale(gyroScaleFloat)>degrees){//gyro negative
			error = fabs(gyroScale(gyroScaleFloat)) - abs(degrees);
			motor[ldt1] = motor[ldt2] = power;//motors positive
			motor[rdt1] = motor[rdt2] = power;
		}
		motor[ldt1] = motor[ldt2] = -power/8;
		motor[rdt1] = motor[rdt2] = -power/8;
		wait1Msec(50);
		motor[ldt1] = motor[ldt2] = 0;
		motor[rdt1] = motor[rdt2] = 0;
	}else if(direction == 0){//if turn is counterclockwise
		while(gyroScale(gyroScaleFloat)<degrees){//gyro positive
			error = -degrees + gyroScale(gyroScaleFloat);
			motor[ldt1] = motor[ldt2] = -power;//motors negative
			motor[rdt1] = motor[rdt2] = -power;
		}
		motor[ldt1] = motor[ldt2] = power/8;
		motor[rdt1] = motor[rdt2] = power/8;
		wait1Msec(50);
		motor[ldt1] = motor[ldt2] = 0;
		motor[rdt1] = motor[rdt2] = 0;
	}
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void mgmForwards(int distance){//this moves the robot forwards *distance* inches while lowering the mgm
	int encVal = getEncValForDistance(distance);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	while(!(SensorValue[rdtEnc]<(-encVal+10) && SensorValue[rdtEnc]>(-encVal-10))){
		motor[ldt1]=motor[ldt2]=90;
		motor[rdt1]=motor[rdt2]=-90;
		motor[mgm]=127;
	}
	motor[ldt1]=motor[ldt2]=0;
	motor[rdt1]=motor[rdt2]=0;
	motor[mgm]=0;
}
void mgmLeftForwards(int distance){//this moves the robot forwards *distance* inches while lowering the mgm
	int encVal = getEncValForDistance(distance);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	while(!(SensorValue[rdtEnc]<(-encVal+10) && SensorValue[rdtEnc]>(-encVal-10))){
		motor[ldt1]=motor[ldt2]=70;
		motor[rdt1]=motor[rdt2]=-120;
		motor[mgm]=127;
	}
	motor[ldt1]=motor[ldt2]=0;
	motor[rdt1]=motor[rdt2]=0;
	motor[mgm]=0;
}

void mgmRightForwards(int distance){//this moves the robot forwards *distance* inches while lowering the mgm
	int encVal = getEncValForDistance(distance);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	while(!(SensorValue[rdtEnc]<(-encVal+10) && SensorValue[rdtEnc]>(-encVal-10))){
		motor[ldt1]=motor[ldt2]=127;
		motor[rdt1]=motor[rdt2]=-70;
		motor[mgm]=127;
	}
	motor[ldt1]=motor[ldt2]=0;
	motor[rdt1]=motor[rdt2]=0;
	motor[mgm]=0;
}
void moveBackwards(int distance){//this moves the robot backwards *distance* inches
	int encVal = getEncValForDistance(distance);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	while(!(SensorValue[rdtEnc]<(encVal+10) && SensorValue[rdtEnc]>(encVal-10))){
			motor[ldt1]=motor[ldt2]=-90;
			motor[rdt1]=motor[rdt2]=90;
	}
	motor[ldt1]=motor[ldt2]=0;
	motor[rdt1]=motor[rdt2]=0;
}
void turnRight(int degrees){//this turns the robot to the right *degrees* degrees
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	motor[ldt1] = motor[ldt2] = C_motorPower;
	motor[rdt1] = motor[rdt2] = C_motorPower;
	wait1Msec((15*1000)/(7*degrees));
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void turnLeft(int degrees){//this turns the robot to the left *degrees* degrees
	motor[ldt1] = motor[ldt2] = -C_motorPower;
	motor[rdt1] = motor[rdt2] = -C_motorPower;
	wait1Msec((15*1000)/(7*degrees));
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void rightAlign(int time, int power){
		motor[ldt1] = motor[ldt2] = power;
		motor[rdt1] = motor[rdt2] = power;
		wait1Msec(time);
		motor[ldt1] = motor[ldt2] = 0;
		motor[rdt1] = motor[rdt2] = 0;
}
void leftAlign(int time, int power){
		motor[ldt1] = motor[ldt2] = -power;
		motor[rdt1] = motor[rdt2] = -power;
		wait1Msec(time);
		motor[ldt1] = motor[ldt2] = 0;
		motor[rdt1] = motor[rdt2] = 0;
}

void lowerMGM(){//lowers mgm
	motor[mgm] = 127;
	wait1Msec(1200);
	motor[mgm] = 20;
}
void raiseMGM(){//raises mgm
	motor[mgm] = -127;
	wait1Msec(800);
	motor[mgm] = 0;
}

void rotateDr4bUpTo(float distance){//rotates double reverse fourbar up to *distance* height
	int encVal = abs(distance*C_dr4bconstant);
	SensorValue[rdr4bEnc]=0;
	SensorValue[ldr4bEnc]=0;
	while(SensorValue[ldr4bEnc]>-encVal && SensorValue[rdr4bEnc]<encVal){
		motor[ldr4b] = C_motorPower;
		motor[rdr4b] = -C_motorPower;
		encVal = abs(distance*C_dr4bconstant);
	}
	motor[ldr4b] = motor[rdr4b] = 0;
}
void rotateDr4bDownTo(float distance){//rotates double reverse fourbar down to *distance* height
	SensorValue[rdr4bEnc]=0;
	SensorValue[ldr4bEnc]=0;
	int encVal = abs(distance*C_dr4bconstant);
	while(SensorValue[ldr4bEnc]<encVal && SensorValue[rdr4bEnc]>-encVal){
		motor[ldr4b] = -C_motorPower;
		motor[rdr4b] = C_motorPower;
		encVal = abs(distance*C_dr4bconstant);
	}
	motor[ldr4b] = motor[rdr4b] = 0;
}
void runDR4BUpFor(int time, int power){
	motor[ldr4b]=power;
	motor[rdr4b]=-power;
	wait1Msec(time);
	motor[ldr4b]=0;
	motor[rdr4b]=0;
}

void runDR4BDownFor(int time, int power){
	motor[ldr4b]=-power;
	motor[rdr4b]=power;
	wait1Msec(time);
	motor[ldr4b]=0;
	motor[rdr4b]=0;
}

void intake(){//intakes the cone
	motor[claw] = 127;
}
void outtake(int time){ //shoots out the code
	motor[claw] = -127;
	wait1Msec(time);
	motor[claw] = 0;
}

void rotateFourbarTo(int position){
	if(SensorValue[fourbarPot]<position){//if the current position is less than desired
		motor[fourbar] = -C_motorPower;
		while(SensorValue[fourbarPot]<position) wait1Msec(5);
		motor[fourbar] = 0;
	}else if(SensorValue[fourbarPot]>position){//if the current position is greater than desired
		motor[fourbar] = C_motorPower;
		while(SensorValue[fourbarPot]>position) wait1Msec(5);
		motor[fourbar] = 0;
	}
}
void rotateFourbarTo(int position, int power){
	if(SensorValue[fourbarPot]<position){//if the current position is less than desired
		motor[fourbar] = -power;
		while(SensorValue[fourbarPot]<position) wait1Msec(5);
		motor[fourbar] = 0;
	}else if(SensorValue[fourbarPot]>position){//if the current position is greater than desired
		motor[fourbar] = power;
		while(SensorValue[fourbarPot]>position) wait1Msec(5);
		motor[fourbar] = 0;
	}
}
void run4BUpFor(int time, int power){
	motor[fourbar]=power;
	wait1Msec(time);
	motor[rdr4b]=0;
}

void run4BDownFor(int time, int power){
	motor[fourbar]=-power;
	wait1Msec(time);
	motor[rdr4b]=0;
}

void robotInit(const string Auton){
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDString(0,0, Auton); //Left Mgm
	displayLCDString(1,0, "is running!");

	motor[claw]=50;
	run4BUpFor(100, 127);
	run4BDownFor(100, 127);
	run4BUpFor(700, 127);
	runDR4BUpFor(100, 20);
	motor[ldt1] = motor[ldt2] = 50;
	motor[rdt1] = motor[rdt2] = -50;
	wait1Msec(100);
	motor[mgm]=50;
}

task auton(){//main task
	getEncValForTurn(1);
	switch(lcdCount){
		case 0:
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0,0, zerothAutonString);//stationary
			displayLCDString(1,0, "is running!");
			motor[claw]=50;
			run4BUpFor(700, 127);
			runDR4BUpFor(350, 100);
			wait1Msec(300);
			motor[ldt1] = motor[ldt2] = 50;
			motor[rdt1] = motor[rdt2] = -50;
			wait1Msec(100);
			motor[ldr4b]=10;
			motor[rdr4b]=-10;
			moveForwards(5);
			wait1Msec(300);
			run4BDownFor(150,50);
			wait1Msec(300);
			outtake(350);
			motor[ldr4b]=0;
			motor[rdr4b]=0;
			wait1Msec(50);
			run4BUpFor(150, 70);
			motor[fourbar]=0;
			moveBackwards(2);
			break;
		case 1:
			robotInit(firstAutonString);
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmLeftForwards(48.5);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			run4BDownFor(20,20);
			wait1Msec(200);
			outtake(300);
			rightAlign(125, 50);
			setStripColor(120, 31, 0, 255, 0);
			//Cone 2
			motor[claw]=127; //intake
			moveForwards(0.75);
			//runDR4BDownFor(200, 100);
			motor[ldr4b]=-20;
			motor[rdr4b]=20;
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(700, 90);
			run4BDownFor(20,20);
			wait1Msec(250);
			outtake(300);
			setStripColor(120, 31, 0, 255, 255);
			//Cone 3
			motor[claw]=127; //intake
			run4BDownFor(150, 127);
			moveForwards(4);
			motor[ldr4b]=-30;
			motor[rdr4b]=30;
			run4BDownFor(500, 127);
			wait1Msec(100);
			run4BUpFor(100, 90);
			runDR4BUpFor(100, 127);
			run4BUpFor(500, 127);
			wait1Msec(300);
			outtake(300);
			setStripColor(120, 31, 0, 0, 255);
			//place mgm
			motor[claw]=-127;
			moveBackwards(54);
			//rightAlign(725, C_motorPower);
			gyroTurn(-130, 80);
			moveForwards(12);
			setStripColor(120, 31, 0, 0, 255);
			//rightAlign(325, C_motorPower);
			gyroTurn(-225, 70);
			//run4BUpFor(100, 90);
			moveForwards(2);
			mgmForwards(25);
			wait1Msec(400);
			moveBackwards(15);
			setStripColor(120, 31, 255, 0, 0);
			break;
		case 2:
			robotInit(secondAutonString);
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmLeftForwards(48.5);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(150);
			outtake(300); //Cone 1/Preload
			rightAlign(175, 50);
			setStripColor(120, 31, 0, 255, 0);
			//Cone 2
			motor[claw]=127; //intake
			moveForwards(1);
			motor[ldr4b]=-20;
			motor[rdr4b]=20;
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(700, 90);
			wait1Msec(200);
			outtake(300);
			setStripColor(120, 31, 0, 255, 255);
			motor[claw]=-127;
			moveBackwards(45);
			rightAlign(850, C_motorPower);
			moveForwards(14);
			setStripColor(120, 31, 0, 0, 255);
			rightAlign(450, C_motorPower);
			moveForwards(8);
			mgmForwards(20);
			moveBackwards(15);
			setStripColor(120, 31, 255, 0, 0);
			break;
		case 3:
			robotInit(thirdAutonString);
			//Cone 1
			motor[ldt1] = motor[ldt2] = -30;//realign
			motor[rdt1] = motor[rdt2] = -30;
			wait1Msec(80);
			mgmForwards(48.5);
			raiseMGM();
			wait1Msec(200);
			outtake(300); //Cone 1/Preload
			//place mgm
			moveBackwards(48);
			wait1Msec(200);
			rightAlign(800, C_motorPower);
			wait1Msec(200);
			moveForwards(10);
			rightAlign(420, C_motorPower);
			motor[claw] = 0;
			wait1Msec(200);
			//run4BUpFor(50, 90);
			moveForwards(8);
			mgmForwards(20);
			moveBackwards(15);
			break;
		case 4:
			robotInit(fourthAutonString);
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmLeftForwards(48.5);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(200);
			outtake(300); //Cone 1/Preload
			rightAlign(170, 49);
			setStripColor(120, 31, 0, 255, 0);
			//Cone 2
			motor[claw]=127; //intake
			moveForwards(1);
			//runDR4BDownFor(200, 100);
			motor[ldr4b]=-20;
			motor[rdr4b]=20;
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(700, 90);
			wait1Msec(250);
			outtake(300);
			setStripColor(120, 31, 0, 255, 255);
			//Cone 3
			motor[claw]=127; //intake
			run4BDownFor(150, 127);
			moveForwards(4);
			motor[ldr4b]=-30;
			motor[rdr4b]=30;
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(100, 90);
			runDR4BUpFor(100, 127);
			run4BUpFor(500, 127);
			wait1Msec(300);
			outtake(300);
			setStripColor(120, 31, 0, 0, 255);
			motor[claw] = 0;
			//place mgm in 10 pt zone
			moveBackwards(54);
			rightAlign(800, C_motorPower);
			moveForwards(10);
			setStripColor(120, 31, 0, 0, 255);
			rightAlign(400, C_motorPower);
			setStripColor(120, 31, 255, 0, 0);
			run4BUpFor(100, 90);
			mgmForwards(10);
			setStripColor(120, 15, 255, 0, 255);
			moveBackwards(6);
			setStripColor(120, 10, 255, 255, 255);
			raiseMGM();
			break;
		case 5:
			robotInit(fifthAutonString);
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmForwards(43);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			run4BDownFor(20,20);
			wait1Msec(200);
			outtake(300); //Cone 1/Preload
			setStripColor(120, 31, 0, 255, 0);
			//rightAlign(175, 50);
			//Cone 2
			motor[claw]=127; //intake
			//runDR4BDownFor(200, 100);
			motor[ldr4b]=-20;
			motor[rdr4b]=20;
			run4BDownFor(550, 127);
			wait1Msec(200);
			run4BUpFor(700, 90);
			wait1Msec(250);
			outtake(300);
			setStripColor(120, 31, 0, 255, 255);
			//Cone 3
			motor[claw]=127; //intake
			run4BDownFor(200, 127);
			moveForwards(3);
			motor[ldr4b]=-30;
			motor[rdr4b]=30;
			run4BDownFor(600, 127);
			wait1Msec(100);
			run4BUpFor(100, 90);
			runDR4BUpFor(100, 127);
			run4BUpFor(500, 127);
			wait1Msec(300);
			outtake(300);
			setStripColor(120, 31, 0, 0, 255);
			//place mgm
			motor[claw]=0;
			moveBackwards(58);
			gyroTurn(135, 80);
			moveForwards(9.5);
			setStripColor(120, 31, 0, 0, 255);
			gyroTurn(225, 80);
			//run4BUpFor(100, 90);
			runDR4BUpFor(100,20);
			moveForwards(12);
			mgmForwards(20);
			moveBackwards(15);
			setStripColor(120, 31, 255, 0, 0);
			break;
		case 6:
			robotInit(sixthAutonString);
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmRightForwards(47);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(150);
			outtake(300); //Cone 1/Preload
			leftAlign(175, 50);
			setStripColor(120, 31, 0, 255, 0);
			//Cone 2
			motor[claw]=127; //intake
			moveForwards(1);
			motor[ldr4b]=-20;
			motor[rdr4b]=20;
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(700, 90);
			wait1Msec(200);
			outtake(300);
			setStripColor(120, 31, 0, 255, 255);
			motor[claw]=-127;
			moveBackwards(55);
			leftAlign(800, C_motorPower);
			moveForwards(14);
			setStripColor(120, 31, 0, 0, 255);
			leftAlign(450, C_motorPower);
			moveForwards(10);
			mgmForwards(20);
			moveBackwards(15);
			setStripColor(120, 31, 255, 0, 0);
			break;
		case 7:
			robotInit(seventhAutonString);
			//Cone 1
			mgmForwards(46.5);
			raiseMGM();
			wait1Msec(200);
			outtake(300); //Cone 1/Preload
			//place mgm
			moveBackwards(44);
			wait1Msec(200);
			leftAlign(800, C_motorPower);
			wait1Msec(200);
			moveForwards(11);
			leftAlign(420, C_motorPower);
			motor[claw] = 0;
			wait1Msec(200);
			//run4BUpFor(50, 90);
			moveForwards(32);
			moveBackwards(15);
			break;
		case 8:
			robotInit(eighthAutonString);
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmForwards(46);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(200);
			outtake(300); //Cone 1/Preload
			setStripColor(120, 31, 0, 255, 0);
			//rightAlign(175, 50);
			//Cone 2
			motor[claw]=127; //intake
			moveForwards(1);
			//runDR4BDownFor(200, 100);
			motor[ldr4b]=-20;
			motor[rdr4b]=20;
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(700, 90);
			wait1Msec(250);
			outtake(300);
			setStripColor(120, 31, 0, 255, 255);
			//Cone 3
			motor[claw]=127; //intake
			run4BDownFor(150, 127);
			moveForwards(4);
			motor[ldr4b]=-30;
			motor[rdr4b]=30;
			run4BDownFor(500, 127);
			wait1Msec(100);
			run4BUpFor(100, 90);
			runDR4BUpFor(100, 127);
			run4BUpFor(500, 127);
			wait1Msec(300);
			outtake(300);
			setStripColor(120, 31, 0, 0, 255);
			//place mgm
			motor[claw]=0;
			moveBackwards(65);
			leftAlign(800, C_motorPower);
			moveForwards(9.5);
			setStripColor(120, 31, 0, 0, 255);
			leftAlign(450,C_motorPower);
			//run4BUpFor(100, 90);
			mgmForwards(10);
			moveBackwards(15);
			setStripColor(120, 31, 255, 0, 0);
			break;
		case 9:
			robotInit(ninthAutonString);
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmRightForwards(47);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(200);
			outtake(300);
		//	leftAlign(175, 40);
			setStripColor(120, 31, 0, 255, 0);
			//Cone 2
			motor[claw]=127; //intake
			moveForwards(1);
			//runDR4BDownFor(200, 100);
			motor[ldr4b]=-20;
			motor[rdr4b]=20;
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(700, 90);
			wait1Msec(250);
			outtake(300);
			setStripColor(120, 31, 0, 255, 255);
			//Cone 3
			motor[claw]=127; //intake
			run4BDownFor(150, 127);
			moveForwards(4);
			motor[ldr4b]=-30;
			motor[rdr4b]=30;
			run4BDownFor(500, 127);
			wait1Msec(100);
			run4BUpFor(100, 90);
			runDR4BUpFor(100, 127);
			run4BUpFor(500, 127);
			wait1Msec(300);
			outtake(300);
			setStripColor(120, 31, 0, 0, 255);
			//place mgm
			motor[claw]=-127;
			moveBackwards(54);
			leftAlign(850, C_motorPower);
			moveForwards(18);
			setStripColor(120, 31, 0, 0, 255);
			leftAlign(400, C_motorPower);
			//run4BUpFor(100, 90);
			moveForwards(30);
			setStripColor(120, 31, 255, 0, 0);
			break;
		case 10:
			robotInit(tenthAutonString);
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmRightForwards(47);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(150);
			outtake(300); //Cone 1/Preload
			//leftAlign(175, 50);
			setStripColor(120, 31, 0, 255, 0);
			//Cone 2
			motor[claw]=127; //intake
			moveForwards(1);
			motor[ldr4b]=-20;
			motor[rdr4b]=20;
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(700, 90);
			wait1Msec(200);
			outtake(300);
			setStripColor(120, 31, 0, 255, 255);
			motor[claw]=-127;
			moveBackwards(45);
			leftAlign(850, C_motorPower);
			moveForwards(16);
			setStripColor(120, 31, 0, 0, 255);
			leftAlign(400, C_motorPower);
			moveForwards(8);
			mgmForwards(20);
			moveBackwards(15);
			setStripColor(120, 31, 255, 0, 0);
			break;
		case 11:
			robotInit(eleventhAutonString);
			//Cone 1
			motor[ldt1] = motor[ldt2] = 30;//realign
			motor[rdt1] = motor[rdt2] = 30;
			wait1Msec(80);
			mgmForwards(50);
			raiseMGM();
			wait1Msec(200);
			outtake(300); //Cone 1/Preload
			//place mgm
			moveBackwards(47);
			wait1Msec(200);
			leftAlign(800, C_motorPower);
			wait1Msec(200);
			moveForwards(11);
			leftAlign(420, C_motorPower);
			motor[claw] = 0;
			wait1Msec(200);
			//run4BUpFor(50, 90);
			moveForwards(8);
			mgmForwards(20);
			moveBackwards(15);
			break;
		case 12:
			robotInit(twelfthAutonString);
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmRightForwards(48.5);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(200);
			outtake(300); //Cone 1/Preload
			//leftAlign(170, 49);
			setStripColor(120, 31, 0, 255, 0);
			//Cone 2
			motor[claw]=127; //intake
			moveForwards(1);
			//runDR4BDownFor(200, 100);
			motor[ldr4b]=-20;
			motor[rdr4b]=20;
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(700, 90);
			wait1Msec(250);
			outtake(300);
			setStripColor(120, 31, 0, 255, 255);
			//Cone 3
			motor[claw]=127; //intake
			run4BDownFor(150, 127);
			moveForwards(4);
			motor[ldr4b]=-30;
			motor[rdr4b]=30;
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(100, 90);
			runDR4BUpFor(100, 127);
			run4BUpFor(500, 127);
			wait1Msec(300);
			outtake(300);
			setStripColor(120, 31, 0, 0, 255);
			motor[claw] = 0;
			//place mgm in 10 pt zone
			moveBackwards(54);
			leftAlign(800, C_motorPower);
			moveForwards(10);
			setStripColor(120, 31, 0, 0, 255);
			leftAlign(400, C_motorPower);
			setStripColor(120, 31, 255, 0, 0);
			run4BUpFor(100, 90);
			mgmForwards(10);
			setStripColor(120, 15, 255, 0, 255);
			moveBackwards(6);
			setStripColor(120, 10, 255, 255, 255);
			raiseMGM();
			break;
		case 13:
			robotInit(thirteenthAutonString);
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmForwards(46);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(200);
			outtake(300); //Cone 1/Preload
			setStripColor(120, 31, 0, 255, 0);
			//rightAlign(175, 50);
			//Cone 2
			motor[claw]=127; //intake
			moveForwards(1);
			//runDR4BDownFor(200, 100);
			motor[ldr4b]=-20;
			motor[rdr4b]=20;
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(700, 90);
			wait1Msec(250);
			outtake(300);
			setStripColor(120, 31, 0, 255, 255);
			//Cone 3
			motor[claw]=127; //intake
			run4BDownFor(150, 127);
			moveForwards(4);
			motor[ldr4b]=-30;
			motor[rdr4b]=30;
			run4BDownFor(500, 127);
			wait1Msec(100);
			run4BUpFor(100, 90);
			runDR4BUpFor(100, 127);
			run4BUpFor(500, 127);
			wait1Msec(300);
			outtake(300);
			setStripColor(120, 31, 0, 0, 255);
			//place mgm
			motor[claw]=0;
			moveBackwards(65);
			rightAlign(800, C_motorPower);
			moveForwards(9.5);
			setStripColor(120, 31, 0, 0, 255);
			rightAlign(450,C_motorPower);
			//run4BUpFor(100, 90);
			moveForwards(10);
			mgmForwards(22);
			moveBackwards(15);
			setStripColor(120, 31, 255, 0, 0);
			break;
		case 14:
			robotInit(fourteenthAutonString);
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmForwards(47);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(150);
			outtake(300); //Cone 1/Preload
			//rightAlign(175, 50);
			setStripColor(120, 31, 0, 255, 0);
			//Cone 2
			motor[claw]=127; //intake
			moveForwards(1);
			motor[ldr4b]=-20;
			motor[rdr4b]=20;
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(700, 90);
			wait1Msec(200);
			outtake(300);
			setStripColor(120, 31, 0, 255, 255);
			motor[claw]=-127;
			moveBackwards(55);
			rightAlign(800, C_motorPower);
			moveForwards(14);
			setStripColor(120, 31, 0, 0, 255);
			rightAlign(450, C_motorPower);
			moveForwards(10);
			mgmForwards(20);
			moveBackwards(15);
			setStripColor(120, 31, 255, 0, 0);
			break;
		case 15:
			robotInit(fifteenthAutonString);
			//Cone 1
			mgmForwards(46.5);
			raiseMGM();
			wait1Msec(200);
			outtake(300); //Cone 1/Preload
			//place mgm
			moveBackwards(44);
			wait1Msec(200);
			rightAlign(800, C_motorPower);
			wait1Msec(200);
			moveForwards(11);
			rightAlign(420, C_motorPower);
			motor[claw] = 0;
			wait1Msec(200);
			//run4BUpFor(50, 90);
			moveForwards(32);
			moveBackwards(15);
			break;
		case 16:
			robotInit(sixteenthAutonString);
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmForwards(46);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(200);
			outtake(300); //Cone 1/Preload
			setStripColor(120, 31, 0, 255, 0);
			//rightAlign(175, 50);
			//Cone 2
			motor[claw]=127; //intake
			moveForwards(1);
			//runDR4BDownFor(200, 100);
			motor[ldr4b]=-20;
			motor[rdr4b]=20;
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(700, 90);
			wait1Msec(250);
			outtake(300);
			setStripColor(120, 31, 0, 255, 255);
			//Cone 3
			motor[claw]=127; //intake
			run4BDownFor(150, 127);
			moveForwards(4);
			motor[ldr4b]=-30;
			motor[rdr4b]=30;
			run4BDownFor(500, 127);
			wait1Msec(100);
			run4BUpFor(100, 90);
			runDR4BUpFor(100, 127);
			run4BUpFor(500, 127);
			wait1Msec(300);
			outtake(300);
			setStripColor(120, 31, 0, 0, 255);
			//place mgm
			motor[claw]=0;
			moveBackwards(65);
			rightAlign(800, C_motorPower);
			moveForwards(9.5);
			setStripColor(120, 31, 0, 0, 255);
			rightAlign(450,C_motorPower);
			//run4BUpFor(100, 90);
			mgmForwards(10);
			moveBackwards(15);
			setStripColor(120, 31, 255, 0, 0);
			break;
		case 17:
			robotInit(seventeenthAutonString);
			motor[ldt1]=motor[ldt2]=90;
			motor[rdt1]=motor[rdt2]=-90;
			wait1Msec(10000);
			motor[ldt1]=motor[ldt2]=-60;
			motor[rdt1]=motor[rdt2]=60;
			wait1Msec(2000);
			motor[ldt1]=motor[ldt2]=0;
			motor[rdt1]=motor[rdt2]=0;
			break;
		case 18:
			//robotInit(eighteenthAutonString);
			moveForwards(5);
			gyroTurn(90,50);
			moveForwards(5);
			gyroTurn(180,50);
			moveForwards(5);
			gyroTurn(270,50);
			moveForwards(5);
			gyroTurn(360,50);
			break;
		default:
			break;
	}
}

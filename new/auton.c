#include "main.c"
#include "driving.c"
#include "lights.c"
#include "gyroLib.c"
const int enterString[] = {247,32,32,32,32,32,69,110,116,101,114,32,32,32,32,246};//this is "�??     Enter    �?�"

const string firstAutonString = "Left 3 20";
const string secondAutonString = "Right 3 20";
const string thirdAutonString = "Stationary Goal";
const string fourthAutonString = "Left 1 20";
const string fifthAutonString = "Left 3 10";
const string sixthAutonString = "Left 1 20";
const string seventhAutonString = "Left 2 20";
const string eightAutonString = "Left 3 5";

int lcdCount = 0;

const int C_rOfWheels = 2;
const int C_rOfRobot = 13;
const float C_PI = 3.1415926;
const int C_motorPower = 127;
const float C_dr4bconstant = 2.5;

//for left encoder, forwards is pos, backwards is neg
//for right encoder, forwards is neg, backwards is pos

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
void gyroTurn(int degrees){
	if(degrees>0){//if turn is clockwise
		motor[ldt1] = motor[ldt2] = C_motorPower;//motors positive
		motor[rdt1] = motor[rdt2] = C_motorPower;
	}else if(degrees<0){//if turn is counterclockwise
		motor[ldt1] = motor[ldt2] = -C_motorPower;//motors negative
		motor[rdt1] = motor[rdt2] = -C_motorPower;
	}
	if(GyroAngleAbsGet() < abs(degrees) && GyroValidGet()) wait1Msec(5);
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
		motor[ldt1]=motor[ldt2]=80;
		motor[rdt1]=motor[rdt2]=-127;
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
		motor[rdt1]=motor[rdt2]=-80;
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
void robotInit(){
	motor[claw]=50;
	run4BUpFor(700, 127);
	runDR4BUpFor(100, 20);
	motor[ldt1] = motor[ldt2] = 50;
	motor[rdt1] = motor[rdt2] = -50;
	wait1Msec(100);
	motor[mgm]=50;
}
task auton(){//main task
	getEncValForTurn(1);
	//GyroInit(in2);
	switch(lcdCount){
		case 0:
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0,0, firstAutonString); //Left Mgm
			displayLCDString(1,0, "is running!");
			robotInit();
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmLeftForwards(50);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(150);
			outtake(300); //Cone 1/Preload
			motor[ldt1] = motor[ldt2] = 50;
			motor[rdt1] = motor[rdt2] = 50;
			wait1Msec(150);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
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
			wait1Msec(200);
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
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(850);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			moveForwards(14);
			setStripColor(120, 31, 0, 0, 255);
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(550);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			//run4BUpFor(100, 90);
			moveForwards(33);
			moveBackwards(15);
			setStripColor(120, 31, 255, 0, 0);
			break;
		case 1:
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0,0, secondAutonString);//Right Mgm
			displayLCDString(1,0, "is running!");
			robotInit();
			//Cone 1
			mgmRightForwards(46.5);
			raiseMGM();
			wait1Msec(150);
			outtake(300); //Cone 1/Preload
			motor[ldt1] = motor[ldt2] = 40;
			motor[rdt1] = motor[rdt2] = 40;
			wait1Msec(120);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			//Cone 2
			moveForwards(1);
			motor[claw]=127; //intake
			runDR4BDownFor(200, 100);
			motor[ldr4b]=-30;
			motor[rdr4b]=30;
			run4BDownFor(550, 127);
			wait1Msec(100);
			run4BUpFor(750, 127);
			wait1Msec(100);
			outtake(300);
			//Cone 3
			motor[claw]=127; //intake
			run4BDownFor(150, 127);
			moveForwards(3);
			runDR4BDownFor(100,50);
			run4BDownFor(500, 127);
			wait1Msec(200);
			run4BUpFor(100, 90);
			runDR4BUpFor(100, 127);
			run4BUpFor(500, 127);
			wait1Msec(300);
			outtake(300);
			//place mgm
			moveBackwards(62);
			wait1Msec(200);
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(300);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			moveBackwards(8);
			wait1Msec(200);
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(350);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			motor[claw] = 0;
			run4BUpFor(50, 90);
			moveForwards(32);
			moveBackwards(15);
			break;
		case 2: //stationary
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0,0, thirdAutonString);
			displayLCDString(1,0, "is running!");
			motor[claw]=50;
			run4BUpFor(700, 127);
			runDR4BUpFor(350, 100);
			wait1Msec(300);
			motor[ldr4b]=10;
			motor[rdr4b]=-10;
			moveForwards(6);
			wait1Msec(300);
			run4BDownFor(150,60);
			wait1Msec(400);
			outtake(300);
			motor[ldr4b]=0;
			motor[rdr4b]=0;
			run4BUpFor(150, 70);
			moveBackwards(2);
			break;
		case 3:
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0,0, fourthAutonString); //preload in the 20 pt zone
			displayLCDString(1,0, "is  running!");
			robotInit();

			//Cone 1
			motor[ldt1] = motor[ldt2] = -30;//realign
			motor[rdt1] = motor[rdt2] = -30;
			wait1Msec(80);
			mgmForwards(46.5);
			raiseMGM();
			wait1Msec(150);
			outtake(300); //Cone 1/Preload
			/*motor[ldt1] = motor[ldt2] = 40;//realign
			motor[rdt1] = motor[rdt2] = 40;
			wait1Msec(120);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;*/
			//place mgm
			moveBackwards(44);
			wait1Msec(200);
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(800);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			wait1Msec(200);
			moveForwards(11);
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(420);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			motor[claw] = 0;
			wait1Msec(200);
			//run4BUpFor(50, 90);
			moveForwards(32);
			moveBackwards(15);
			break;
		case 4:
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0,0, fifthAutonString); //Left Mgm 3 cones in the 10 pt zone
			displayLCDString(1,0, "is  running!");
			robotInit();
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmLeftForwards(50);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(150);
			outtake(300); //Cone 1/Preload
			motor[ldt1] = motor[ldt2] = 50;
			motor[rdt1] = motor[rdt2] = 50;
			wait1Msec(150);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
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
			wait1Msec(200);
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
			//place mgm in 10 pt zone
			motor[claw]=-127;
			moveBackwards(54);
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(800);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			moveForwards(4);
			setStripColor(120, 31, 0, 0, 255);
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(500);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			setStripColor(120, 31, 255, 0, 0);
			run4BUpFor(100, 90);
			mgmForwards(7.5);
			lowerMGM();
			setStripColor(120, 15, 255, 0, 255);
			moveBackwards(6);
			setStripColor(120, 10, 255, 255, 255);
			raiseMGM();
			break;
		case 5:
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0,0, sixthAutonString);
			displayLCDString(1,0, "is running!");
			robotInit();
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmLeftForwards(50);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(150);
			outtake(300); //Cone 1/Preload
			motor[ldt1] = motor[ldt2] = 50;
			motor[rdt1] = motor[rdt2] = 50;
			wait1Msec(150);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			setStripColor(120, 31, 0, 255, 0);
			motor[claw]=-127;
			moveBackwards(45);
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(850);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			moveForwards(14);
			setStripColor(120, 31, 0, 0, 255);
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(450);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			//run4BUpFor(100, 90);
			moveForwards(33);
			moveBackwards(15);
			setStripColor(120, 31, 255, 0, 0);
			break;
		case 6:
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0,0, seventhAutonString);
			displayLCDString(1,0, "is running!");
				clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0,0, firstAutonString); //Left Mgm
			displayLCDString(1,0, "is running!");
			robotInit();
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmLeftForwards(50);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(150);
			outtake(300); //Cone 1/Preload
			motor[ldt1] = motor[ldt2] = 50;
			motor[rdt1] = motor[rdt2] = 50;
			wait1Msec(150);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
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
			wait1Msec(200);
			outtake(300);
			setStripColor(120, 31, 0, 255, 255);
			motor[claw]=-127;
			moveBackwards(45);
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(850);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			moveForwards(14);
			setStripColor(120, 31, 0, 0, 255);
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(450);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			//run4BUpFor(100, 90);
			moveForwards(33);
			moveBackwards(15);
			setStripColor(120, 31, 255, 0, 0);
			break;
		case 7:
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0,0, eightAutonString);
			displayLCDString(1,0, "is running!");
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0,0, firstAutonString); //Left Mgm
			displayLCDString(1,0, "is running!");
			robotInit();
			setStripColor(120, 31, 255, 0, 0);
			//Cone 1
			mgmLeftForwards(50);
			raiseMGM();
			setStripColor(120, 31, 255, 255, 0);
			wait1Msec(150);
			outtake(300); //Cone 1/Preload
			motor[ldt1] = motor[ldt2] = 50;
			motor[rdt1] = motor[rdt2] = 50;
			wait1Msec(150);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
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
			wait1Msec(200);
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
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(850);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			moveForwards(14);
			setStripColor(120, 31, 0, 0, 255);
			motor[ldt1] = motor[ldt2] = C_motorPower;
			motor[rdt1] = motor[rdt2] = C_motorPower;
			wait1Msec(450);
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
			//run4BUpFor(100, 90);
			moveForwards(33);
			moveBackwards(15);
			setStripColor(120, 31, 255, 0, 0);
			break;
			default:
			break;
	}
}

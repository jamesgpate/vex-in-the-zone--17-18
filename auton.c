#include "main.c"
#include "driving.c"

const int enterString[] = {247,32,32,32,32,32,69,110,116,101,114,32,32,32,32,246};//this is "�??     Enter    �?�"

const string firstAutonString = "Forward+Backward";
const string secondAutonString = "Left+Right";
const string thirdAutonString = "Stationary Goal";
const string fourthAutonString = "Nothing";

int lcdCount = 0;

const float drivingKP = 0.3;
const float drivingKI = 0.0;
const float drivingKD = 0.0;

const int C_rOfWheels = 2;
const int C_rOfRobot = 13;
const float C_PI = 3.1415926;
const int C_motorPower = 100;
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
void gyroTurn( int degrees){
	//plz setup
}

void moveForwards(int distance){//this moves the robot forwards *distance* inches
	distance = 0.921278810118277*(distance)-0.227827752504;
	int encVal = getEncValForDistance(distance);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	int lastError = encVal;
	int totalError = 0;
	//clearDebugStream();
	while(!(SensorValue[rdtEnc]<(-encVal+10) && SensorValue[rdtEnc]>(-encVal-10))){
		int error = encVal+SensorValue[rdtEnc];
		//if(SensorValue[rdtEnc]<(-encVal+10) && SensorValue[rdtEnc]>(-encVal-10)) error=0;
		if(totalError<50)totalError += error;
		else totalError=0;
		int power = ((drivingKP*error)+(drivingKI*totalError)+(drivingKD*(error-lastError)));
		if(power>90){
			motor[ldt1]=motor[ldt2]=90;
			motor[rdt1]=motor[rdt2]=-90;
		}
		else if(power<15&&power>-15){
			motor[ldt1]=motor[ldt2]=0;
			motor[rdt1]=motor[rdt2]=0;
		}else{
			motor[ldt1]=motor[ldt2]=power;
			motor[rdt1]=motor[rdt2]=-power;
		}
		/*string err = "";
		string kp = "KP=";
		string ki = "KI=";
		string kd = "KD=";

		int kPower = (drivingKP*error);
		int kIower = (drivingKI*totalError);
		int kDower = (drivingKD*(error-lastError));

		err = err + error;
		kp = kp + kPower;
		ki = ki + kIower;
		kd = kd + kDower;

		writeDebugStreamLine(err);
		writeDebugStreamLine(kp);
		writeDebugStreamLine(ki);
		writeDebugStreamLine(kd);*/
		lastError=error;
		wait1Msec(25);
	}
	motor[ldt1]=motor[ldt2]=0;
	motor[rdt1]=motor[rdt2]=0;
}
void moveBackwards(int distance){//this moves the robot backwards *distance* inches
	distance = 0.921278810118277*(distance)-0.227827752504;
	int encVal = getEncValForDistance(distance);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	int lastError = encVal;
	int totalError = 0;
	while(!(SensorValue[rdtEnc]<(encVal+10) && SensorValue[rdtEnc]>(encVal-10))){
		int error = encVal+SensorValue[rdtEnc];
		if(totalError<50)totalError += error;
		else totalError=0;
		int power = ((drivingKP*error)+(drivingKI*totalError)+(drivingKD*(error-lastError)));
		if(power>90){
			motor[ldt1]=motor[ldt2]=-90;
			motor[rdt1]=motor[rdt2]=90;
		}
		if(power<10){
			motor[ldt1]=motor[ldt2]=0;
			motor[rdt1]=motor[rdt2]=0;
		}
		else{
		motor[ldt1]=motor[ldt2]=-power;
		motor[rdt1]=motor[rdt2]=power;
		}
		wait1Msec(25);
	}
	motor[ldt1]=motor[ldt2]=0;
	motor[rdt1]=motor[rdt2]=0;
}
void turnRight(int degrees){//this turns the robot to the right *degrees* degrees
	gyroTurn((float)degrees);
	motor[ldt1] = motor[ldt2] = C_motorPower;
	motor[rdt1] = motor[rdt2] = C_motorPower;
	wait1Msec((15*1000)/(7*degrees));
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void turnLeft(int degrees){//this turns the robot to the left *degrees* degrees
	gyroTurn((float)(-degrees));
	motor[ldt1] = motor[ldt2] = -C_motorPower;
	motor[rdt1] = motor[rdt2] = -C_motorPower;
	wait1Msec((15*1000)/(7*degrees));
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void lowerMGM(){//lowers mgm
	motor[mgm] = 127;
	wait1Msec(1500);
	motor[mgm] = 20;
}
void raiseMGM(){//raises mgm
	motor[mgm] = -C_motorPower;
	wait1Msec(1000);
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
void harvesterUp(){//intakes the cone
	motor[claw] = C_motorPower;
	wait1Msec(500);
	motor[claw] = 0;
}
void harvesterDown(){ //shoots out the code
	motor[claw] = -C_motorPower;
	wait1Msec(1000);
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
task auton(){//main task
	getEncValForTurn(1);
	switch(lcdCount){
		case 0:
			displayLCDString(0,0, firstAutonString);
			displayLCDString(1,0, "is running!");
		motor[claw]=50;
		motor[fourbar]=127;
		wait1Msec(750);
		motor[fourbar]=0;
		moveForwards(36);
			motor[ldr4b]= 127;
			motor[rdr4b]=-127;
			wait1Msec(100);
			motor[ldr4b]=0;
			motor[rdr4b]=0;
			lowerMGM();
			moveForwards(16);
			raiseMGM();
			motor[ldr4b]=-127; //drop dr4b
			motor[rdr4b]=127;
			wait1Msec(75);
			motor[ldr4b]=0;
			motor[rdr4b]=0;
			wait1Msec(200);
			motor[claw]=-127; //outtake preload
			wait1Msec(300);
			motor[claw]=90; //intake
			wait1Msec(0);
			motor[fourbar]=-127; //drop 4b
		  wait1Msec(600);
		  motor[fourbar]=0;
		  motor[ldt1]=motor[ldt2]=50; //drive forward
		  motor[rdt1]=motor[rdt2]=-50;
		  wait1Msec(300);
		  motor[ldt1]=motor[ldt2]=0; //stop driving forward
		  motor[rdt1]=motor[rdt2]=0;
		  wait1Msec(600);
		  motor[fourbar]=127;//lift 4b
			wait1Msec(750);
			motor[fourbar]=0; //stop lifting 4b
			motor[claw]=-127; //outtake cone 2
			wait1Msec(300);
			motor[claw]=127; //intake
			motor[fourbar]=-127; //drop 4b
		  wait1Msec(100);
		  motor[fourbar]=0; //stop dropping 4b
		  motor[ldt1]=motor[ldt2]=50; //drive forward
		  motor[rdt1]=motor[rdt2]=-50;
		  wait1Msec(500);
		  motor[ldt1]=motor[ldt2]=0; //stop driving forward
		  motor[rdt1]=motor[rdt2]=0;
		  motor[fourbar]=-127;//drop 4b
			wait1Msec(400);
			motor[fourbar]=127;//lift 4b
			wait1Msec(500);
			//motor[ldr4b]= 20;//lift dr4b
		  //motor[rdr4b]=-20;
			//wait1Msec(30);
		  //motor[fourbar]=127; //lift 4 bar
			//wait1Msec(400);
			motor[fourbar]=0; //stop lifting 4b
			wait1Msec(750);
			//motor[ldr4b]=-80; //drop dr4b
			//motor[rdr4b]=80;
			//wait1Msec(300);
			motor[claw]=-127; //outtake cone 3
			wait1Msec(300);
			//motor[ldr4b]= 15;//lift dr4b
		  //motor[rdr4b]=-15;
		  //wait1Msec(100);
			motor[fourbar]=-127; //drop 4b to parallel
		  wait1Msec(300);
			motor[ldr4b]=-127;//drop dr4b
			motor[rdr4b]=127;
			wait1Msec(100);
			motor[ldr4b]=0;
			motor[rdr4b]=0;
			motor[claw]=127;
			motor[fourbar]=-127;
		  wait1Msec(400);
		  motor[fourbar]=0;
		 /* moveForwards(6);
		  motor[ldr4b]= 127;
			motor[rdr4b]=-127;
			wait1Msec(200);
			motor[ldr4b]=0;
			motor[rdr4b]=0;
		  motor[fourbar]=127;
			wait1Msec(750);
			motor[fourbar]=0;
			motor[claw]=-127;
			wait1Msec(200);
			motor[ldr4b]= -127;
			motor[rdr4b]=127;
			wait1Msec(200);
			motor[ldr4b]=0;
			motor[rdr4b]=0; */


			//moveBackwards(20);
			break;
		case 1:
			displayLCDString(0,0, secondAutonString);
			displayLCDString(1,0, "is running!");
			turnLeft(20);
			turnRight(20);
			break;
		case 2:
			displayLCDString(0,0, thirdAutonString);
			displayLCDString(1,0, "is running!");
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
			break;
		case 3:
			displayLCDString(0,0, fourthAutonString);
			displayLCDString(1,0, "is running!");
			break;
		default:
			break;
	}
}

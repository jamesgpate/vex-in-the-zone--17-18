int lcdCount = 0;
const int C_dOfWheels = 4;
const int C_rOfRobot = 13;
const float C_PI = 3.1415926;
const int C_motorPower = 70;
const float C_dr4bconstant = 2.714;
const float C_coneHeight = 7;		 // Plz double check
const float C_fourbarRadius = 8.75;  // Plz double check
//
int getEncValForDistance(int inches){//this returns the encoder value for drivetrain distance
	return (360*inches)/(C_dOfWheels*C_PI/2);
}
int getEncValForTurn(int degrees){//this returns how many times an encoder on the drivetrain needs to turn in relation to how far the robot needs to turn
	return (360*C_PI*2*C_rOfRobot)/(360*C_dOfWheels);
}
void moveForwards(int distance){//this moves the robot forwards *distance* inches
	int encVal = getEncValForDistance(distance);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	while(SensorValue[ldtEnc]<encVal || SensorValue[rdtEnc]>-encVal){
		motor[ldt1] = motor[ldt2] = C_motorPower;
		motor[rdt1] = motor[rdt2] = -C_motorPower;
	}
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void moveBackwards(int distance){//this moves the robot backwards *distance* inches
	int encVal = getEncValForDistance(distance);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	while(SensorValue[ldtEnc]>-encVal || SensorValue[rdtEnc]<encVal){
		motor[ldt1] = motor[ldt2] = -C_motorPower;
		motor[rdt1] = motor[rdt2] = C_motorPower;
	}
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void turnRight(int degrees){//this turns the robot to the right *degrees* degrees
	int encVal = getEncValForTurn(degrees);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	while(SensorValue[ldtEnc]<encVal || SensorValue[rdtEnc]>-encVal){
		motor[ldt1] = motor[ldt2] = C_motorPower;
		motor[rdt1] = motor[rdt2] = C_motorPower;
	}
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void turnLeft(int degrees){//this turns the robot to the left *degrees* degrees
	int encVal = getEncValForTurn(degrees);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	while(SensorValue[ldtEnc]>-encVal || SensorValue[rdtEnc]<encVal){
		motor[ldt1] = motor[ldt2] = -C_motorPower;
		motor[rdt1] = motor[rdt2] = -C_motorPower;
	}
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void lowerMGM(){//lowers mgm
	motor[mgml] = motor[mgmr] = C_motorPower;
	wait1Msec(500);
	motor[mgml] = motor[mgmr] = 0;
}
void raiseMGM(){//raises mgm
	motor[mgml] = motor[mgmr] = -C_motorPower;
	wait1Msec(500);
	motor[mgml] = motor[mgmr] = 0;
}
void rotateDr4bUpTo(int distance){//rotates double reverse fourbar up to *distance* height
	int encVal = abs(distance*C_dr4bconstant);
	while(SensorValue[ldr4bEnc]>-encVal || SensorValue[rdr4bEnc]<encVal){
		motor[ldr4b] = -C_motorPower;
		motor[rdr4b] = C_motorPower;
	}
	motor[ldr4b] = motor[rdr4b] = 0;
}
void rotateDr4bDownTo(int distance){//rotates double reverse fourbar down to *distance* height
	int encVal = abs(distance*C_dr4bconstant);
	while(SensorValue[ldr4bEnc]<encVal || SensorValue[rdr4bEnc]>-encVal){
		motor[ldr4b] = C_motorPower;
		motor[rdr4b] = -C_motorPower;
	}
	motor[ldr4b] = motor[rdr4b] = 0;
}
void harvesterUp(){
	motor[claw] = -C_motorPower;
	wait1Msec(500);
	motor[claw] = 0;
}
void harvesterDown(){
	motor[claw] = C_motorPower;
	wait1Msec(500);
	motor[claw] = 0;
}
void rotateFourbarTo(int degrees){
	int fourbarDegValue = (SensorValue[fourbarEnc]%360)*360; //correct
	if(degrees<fourbarDegValue){
		while(degrees<fourbarDegValue){
			motor[fourbar] = C_motorPower;
		}
	}else if(degrees>fourbarDegValue){
		while(degrees>fourbarDegValue){
			motor[fourbar] = -C_motorPower;
		}
	}
	motor[fourbar] = 0;
}
task auton(){//main task
	switch(lcdCount){
		case 0://first auton
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
			break;
		case 3:
			break;
		default:
			break;
	}
}
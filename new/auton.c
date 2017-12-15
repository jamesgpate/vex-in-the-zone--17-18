/*
    Copyright (C) 2017 Quantum Robotics

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
int lcdCount = 0;
const int C_dOfWheels = 4;
const int C_rOfRobot = 8;
const float C_PI = 3.1415926;
const int C_motorPower = 40;
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
	while(SensorValue[ldtEnc]<encVal || SensorValue[rdtEnc]>encVal){
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
	while(SensorValue[ldtEnc]>encVal || SensorValue[rdtEnc]<encVal){
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
	while(SensorValue[ldtEnc]<encVal || SensorValue[rdtEnc]>encVal){
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
	while(SensorValue[ldtEnc]>encVal || SensorValue[rdtEnc]<encVal){
		motor[ldt1] = motor[ldt2] = -C_motorPower;
		motor[rdt1] = motor[rdt2] = -C_motorPower;
	}
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void lowerMGM(){
	motor[mgml] = motor[mgmr] = C_motorPower;
	wait1Msec(500);
	motor[mgml] = motor[mgmr] = 0;
}
void raiseMGM(){
	motor[mgml] = motor[mgmr] = -C_motorPower;
	wait1Msec(500);
	motor[mgml] = motor[mgmr] = 0;
}
task auton(){//main task
	switch(lcdCount){
		case 0://first auton
			moveForwards(60);
			lowerMGM();
			moveForwards(5);
			raiseMGM();
			turnRight(180);
			moveForwards(40);
			turnLeft(45);
			moveForwards(10);
			turnRight(90);
			moveForwards(20);
			lowerMGM();
			moveForwards(5);
			raiseMGM();
			moveBackwards(5);
			break;
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		default:
			break;
	}
}

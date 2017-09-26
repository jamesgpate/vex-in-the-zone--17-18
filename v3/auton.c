#include "main.c"
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
const int DIAMWHEELS = 4;
const int rOfRobot = 8.4;
int count = 0;
int getEncVal(int degrees){//this returns
	float distance = 2*3.14159*rOfRobot;
	float rotations = distance/(DIAMWHEELS*3.14159);
	int encValue = rotations*360;
	return encValue;
}
void moveForwards(int distance){//this moves the robot forwards *distance* inches
	float circumference = DIAMWHEELS*3.14159;
	float rotations = distance/circumference;
	int encValue = rotations*360;
	nMotorEncoder[I2C_1] = 0;
	nMotorEncoder[I2C_2] = 0;
	while(nMotorEncoder[I2C_1] < encValue && nMotorEncoder[I2C_2] < encValue){
		motor[left1] = 127;
		motor[left2] = 127;
		motor[right1] = 127;
		motor[right2] = 127;
	}
	motor[left1] = 0;
	motor[left2] = 0;
	motor[right1] = 0;
	motor[right2] = 0;
}
void moveBackwards(int distance){//this moves the robot backwards *distance* inches
	float circumference = DIAMWHEELS*3.14159;
	float rotations = distance/circumference;
	int encValue = rotations*360;
	nMotorEncoder[I2C_1] = 0;
	nMotorEncoder[I2C_2] = 0;
	while(nMotorEncoder[I2C_1] < encValue && nMotorEncoder[I2C_2] < encValue){
		motor[left1] = -127;
		motor[left2] = -127;
		motor[right1] = -127;
		motor[right2] = -127;
	}
	motor[left1] = 0;
	motor[left2] = 0;
	motor[right1] = 0;
	motor[right2] = 0;
}
void turnRight(int degrees){//this turns the robot to the right *degrees* degrees
	int encValue = getEncVal(degrees);
	while(nMotorEncoder[I2C_1] < encValue && nMotorEncoder[I2C_2] < encValue){
		motor[left1] = 127;
		motor[left2] = 127;
		motor[right1] = -127;
		motor[right2] = -127;
	}
	motor[left1] = 0;
	motor[left2] = 0;
	motor[right1] = 0;
	motor[right2] = 0;
}
void turnLeft(int degrees){//this turns the robot to the left *degrees* degrees
	int encValue = getEncVal(degrees);
	while(nMotorEncoder[I2C_1] < encValue && nMotorEncoder[I2C_2] < encValue){
		motor[left1] = -127;
		motor[left2] = -127;
		motor[right1] = 127;
		motor[right2] = 127;
	}
	motor[left1] = 0;
	motor[left2] = 0;
	motor[right1] = 0;
	motor[right2] = 0;
}
task auton(){
	switch(count){
		case 0://first auton
			displayLCDCenteredString(0, "Autonomous 1");
			displayLCDCenteredString(1, "is running!");
			moveForwards(10);
			turnRight(90);
			moveForwards(5);
			turnLeft(90);
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

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
int getEncVal(int degrees){
	float distance = 2*3.14159*rOfRobot;
	float rotations = distance/(DIAMWHEELS*3.14159);
	int encValue = rotations*360;
	return encValue;
}
void moveForwards(int distance){
	float circumference = DIAMWHEELS*3.14159;
	float rotations = distance/circumference;
	int encValue = rotations*360;
	SensorValue[encRDT] = 0;
	SensorValue[encLDT] = 0;
	while(SensorValue[encRDT] < encValue && SensorValue[encLDT] < encValue){
		motor[backLeft] = 127;
		motor[frontLeft] = 127;
		motor[backRight] = 127;
		motor[frontRight] = 127;
	}
	motor[backLeft] = 0;
	motor[frontLeft] = 0;
	motor[backRight] = 0;
	motor[frontRight] = 0;
	return 0;
}
void moveBackwards(int distance){
	float circumference = DIAMWHEELS*3.14159;
	float rotations = distance/circumference;
	int encValue = rotations*360;
	SensorValue[encRDT] = 0;
	SensorValue[encLDT] = 0;
	while(SensorValue[encRDT] < encValue && SensorValue[encLDT] < encValue){
		motor[backLeft] = -127;
		motor[frontLeft] = -127;
		motor[backRight] = -127;
		motor[frontRight] = -127;
	}
	motor[backLeft] = 0;
	motor[frontLeft] = 0;
	motor[backRight] = 0;
	motor[frontRight] = 0;
	return 0;
}
void turnRight(int degrees){
	int encValue = getEncVal(degrees);
	while(SensorValue[encRDT] < encValue && SensorValue[encLDT] < encValue){
		motor[backLeft] = 127;
		motor[frontLeft] = 127;
		motor[backRight] = -127;
		motor[frontRight] = -127;
	}
	motor[backLeft] = 0;
	motor[frontLeft] = 0;
	motor[backRight] = 0;
	motor[frontRight] = 0;
	return 0;
}
void turnLeft(int degrees){
	int encValue = getEncVal(degrees);
	while(SensorValue[encRDT] < encValue && SensorValue[encLDT] < encValue){
		motor[backLeft] = -127;
		motor[frontLeft] = -127;
		motor[backRight] = 127;
		motor[frontRight] = 127;
	}
	motor[backLeft] = 0;
	motor[frontLeft] = 0;
	motor[backRight] = 0;
	motor[frontRight] = 0;
	return 0;
}
task auton(){
	switch(count){
		case 0:
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
	return 0;
}

#include "main.c"
/*
    Copyright (C) <year>  <name of author>

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
int DIAMWHEELS = 4;
task auton(){

}
void moveForward(int distance){
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
}
void turnRight(int degrees){

}
void turnLeft(int degrees){

}

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
int getEncVal(int degrees){//this returns how many times an encoder on the drivetrain needs to turn in relation to how far the robot needs to turn
	float distance = 2*3.14159*rOfRobot;
	float rotations = distance/(DIAMWHEELS*3.14159);
	return rotations*360;
}
void moveForwards(int distance){//this moves the robot forwards *distance* inches
	float circumference = DIAMWHEELS*3.14159;
	float rotations = distance/circumference;
	int encValue = rotations*360;
	SensorValue[leftDT] = 0;
	SensorValue[rightDT] = 0;
	while(SensorValue[leftDT] < encValue && SensorValue[rightDT] < encValue){
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
	SensorValue[leftDT] = 0;
	SensorValue[rightDT] = 0;
	while(SensorValue[leftDT] < encValue && SensorValue[rightDT] < encValue){
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
	while(SensorValue[leftDT] < encValue && SensorValue[rightDT] < encValue){
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
	while(SensorValue[leftDT] < encValue && SensorValue[rightDT] < encValue){
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
			motor[claw] = 127;
			wait1Msec(500);
			moveForwards(10);
			while(SensorValue[towerL] < 30 && SensorValue[towerR] < 30){
				motor[ldr4b] = 63;
				motor[rdr4b] = -63;
			}
			motor[claw] = -127;
			wait1Msec(500);
			motor[ldr4b] = 0;
			motor[rdr4b] = 0;
			moveForwards(10);
			while(SensorValue[towerL] > 0 && SensorValue[towerR] > 0){
				motor[ldr4b] = -63;
				motor[rdr4b] = 63;
			}
			motor[ldr4b] = 0;
			motor[rdr4b] = 0;
			motor[claw] = 127;
			wait1Msec(500);
			motor[claw] = -127;
			wait1Msec(500);
			motor[claw] = 0;
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

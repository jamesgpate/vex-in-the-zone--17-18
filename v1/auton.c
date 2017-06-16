#include "main.c"
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

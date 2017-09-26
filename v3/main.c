#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, dgtl5,  towerL,         sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  towerR,         sensorQuadEncoder)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           left1,         tmotorVex393TurboSpeed_MC29, openLoop, driveLeft, encoderPort, I2C_1)
#pragma config(Motor,  port3,           right1,        tmotorVex393TurboSpeed_MC29, openLoop, reversed, driveRight, encoderPort, I2C_2)
#pragma config(Motor,  port4,           ldr4b,         tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           left2,         tmotorVex393TurboSpeed_MC29, openLoop, driveLeft)
#pragma config(Motor,  port7,           rdr4b,         tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           claw,          tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           right2,        tmotorVex393TurboSpeed_MC29, openLoop, reversed, driveRight)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

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
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)
#pragma platform(VEX2)
#include "Vex_Competition_Includes.c"
#include "driving.c"
#include "auton.c"
const short leftButton = 1;
const short centerButton = 2;
const short rightButton = 4;
void waitForRelease()
{
	while(nLCDButtons != 0)
		wait1Msec(5);
}
void waitForPress()
{
	while(nLCDButtons == 0)
		wait1Msec(5);
}
void pre_auton(){//Selects auton program
	clearLCDLine(0);
	clearLCDLine(1);
	bLCDBacklight = true;
	while(nLCDButtons != centerButton){
		switch(count){
		case 0:
			displayLCDCenteredString(0, "Autonomous 1");
			displayLCDCenteredString(1, "<         Enter        >");
			waitForPress();
			if(nLCDButtons == leftButton){
				waitForRelease();
				count = 3;
			}
			else if(nLCDButtons == rightButton){
				waitForRelease();
				count++;
			}
		break;
		case 1:
			displayLCDCenteredString(0, "Autonomous 2");
			displayLCDCenteredString(1, "<         Enter        >");
			waitForPress();
			if(nLCDButtons == leftButton){
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton){
				waitForRelease();
				count++;
			}
			break;
		case 2:
			displayLCDCenteredString(0, "Autonomous 3");
			displayLCDCenteredString(1, "<         Enter        >");
			waitForPress();
			if(nLCDButtons == leftButton){
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton){
				waitForRelease();
				count++;
			}
			break;
		case 3:
			displayLCDCenteredString(0, "Autonomous 4");
			displayLCDCenteredString(1, "<         Enter        >");
			waitForPress();
			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count = 0;
			}
			break;
		default:
			count = 0;
			break;

		}
	}
}
task autonomous(){
	startTask(auton);
}
task usercontrol(){
	startTask(drive);
}

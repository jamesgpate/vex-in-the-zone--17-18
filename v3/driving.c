#include "main.c"
#include "pid.c"
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
task drive(){
	bool precision = false;
	int c2 = 0;
	int c3 = 0;
	int lcdRefresh = 0;
	//pid()
	while(true){
		//set threshold to 20 and make sure it is zero under it
		const int THRESHOLD = 20;
		if(abs(vexRT[Ch2])>THRESHOLD)
			c2 = vexRT[Ch2];
		else
			c2 = 0;
		if(abs(vexRT[Ch3])>THRESHOLD)
			c3 = vexRT[Ch3];
		else
			c3 = 0;
		//regular speed
		if(!precision){
			motor[left1] = c3;
			motor[left2] = c3;
			motor[right1] = c2;
			motor[right2] = c2;
		}
		//half speed
		if(precision){
			motor[left1] = c3/2;
			motor[left2] = c3/2;
			motor[right1] = c2/2;
			motor[right2] = c2/2;
		}
		//switch for above
		if(vexRT[Btn8L]==1)
			precision=!precision;
		//6U/6D for dr4b, 5U for half speed
		if(vexRT[Btn6U]==1){
			if(vexRT[Btn5U]==1){
				motor[ldr4b] = 63;
				motor[rdr4b] = 63;
			}else if(vexRT[Btn5U]==0){
				motor[ldr4b] = 127;
				motor[rdr4b] = 127;
			}
		}else if(vexRT[Btn6D]==1){
			if(vexRT[Btn5U]==1){
				motor[ldr4b] = -63;
				motor[rdr4b] = -63;
			}else if(vexRT[Btn5U]==0){
				motor[ldr4b] = -127;
				motor[rdr4b] = -127;
			}
		}else{
			motor[ldr4b] = 0;
			motor[rdr4b] = 0;
		}
		//open and close claw
		if(vexRT[Btn5D]==1)
			motor[claw] = 127;
		else if(vexRT[Btn7D]==1)
			motor[claw] = -63;
		else
			motor[claw] = 0;
		//sounds
		if(!bSoundActive){
			if(vexRT[Btn8R]==1){
				wait1Msec(200);
				if(vexRT[Btn8R]==1){
					playSoundFile("allstar.wav");
				}
			}
			if(vexRT[Btn8U]==1){
				wait1Msec(200);
				if(vexRT[Btn8U]==1){
					playSoundFile("autozone.wav");
				}
			}
		}
		if(lcdRefresh%5==0){//refreshes lcd every tenth of a second
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0,0,"Battery: ");
			displayLCDNumber(0,9, nImmediateBatteryLevel);
			displayLCDString(0,13, " mV");
			displayLCDString(1,0,"Backup: ");
			displayLCDNumber(1,9,BackupBatteryLevel);
			displayLCDString(1,13, " mV");
			wait1Msec(25);
			++lcdRefresh;
		}
	}
}

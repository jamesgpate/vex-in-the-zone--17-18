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
#include "lights.c"
task drive(){
	int c4 = 0, c3 = 0, c2 = 0, c1 = 0;
	while(true){
		clearTimer(T1);
		//set threshold to 20 and make sure it is zero under it
		const int THRESHOLD = 20;
		if(abs(vexRT[Ch4])>THRESHOLD) c4 = vexRT[Ch4];
		else c4 = 0;
		if(abs(vexRT[Ch3])>THRESHOLD) c3 = vexRT[Ch3];
		else c3 = 0;
		if(abs(vexRT[Ch2])>THRESHOLD) c2 = vexRT[Ch2];
		else c2 = 0;
		if(abs(vexRT[Ch1])>THRESHOLD) c1 = vexRT[Ch1];
		else c1 = 0;
		//send these values to the motor
		motor[ldt1] = motor[ldt2] = motor[ldt3] = c3+(c4/2);
		motor[rdt1] = motor[rdt2] = motor[rdt3] = -c3+(c4/2);
		//mobile goal
		if(vexRT[Btn5U])motor[mgml] = motor[mgmr] = -90;
		else if(vexRT[Btn5D])motor[mgml] = motor[mgmr] = 90;
		else motor[mgml] = motor[mgmr] = 0;
		//sounds
		/*if(!bSoundActive){
			if(vexRT[Btn8U]==1){
				wait1Msec(100);
				playSoundFile("allstar.wav");
			}
			if(vexRT[Btn8D]==1){
				wait1Msec(100);
				playSoundFile("omae_wa_mou_shindeiru.wav");
			}
		}*/
		//light functions
		if(vexRT[Btn7U]==1){
			startTask(stopLightTasks);
			startTask(fadeColors);
		}
		if(vexRT[Btn7D]==1){
			startTask(stopLightTasks);
			startTask(smoothWave);
		}
		if(vexRT[Btn8L]==1){
			startTask(stopLightTasks);
			startTask(smoothWaveFullStrip);
		}
		if(vexRT[Btn8R]==1){
			startTask(stopLightTasks);
			startTask(christmasLights);
		}
		//displays current battery and backup battery voltage
		clearLCDLine(0);
		clearLCDLine(1);
		displayLCDString(0,0,"Battery: ");
		displayLCDNumber(0,9, nImmediateBatteryLevel);
		displayLCDString(0,13, " mV");
		displayLCDString(1,0,"Backup: ");
		displayLCDNumber(1,9,BackupBatteryLevel);
		displayLCDString(1,13, " mV");
		wait1Msec(25-time1[T1]);
		EndTimeSlice();
	}
}

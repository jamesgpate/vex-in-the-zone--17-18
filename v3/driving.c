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
	int c4 = 0;
	int c3 = 0;
	while(true){
		//set threshold to 20 and make sure it is zero under it
		const int THRESHOLD = 20;
		if(abs(vexRT[Ch2])>THRESHOLD)
			c4 = vexRT[Ch4];
		else
			c4 = 0;
		if(abs(vexRT[Ch3])>THRESHOLD)
			c3 = vexRT[Ch3];
		else
			c3 = 0;
		//regular speed
		if(!precision){
			motor[fldt] = motor[bldt] = c4+c3;
			motor[frdt] = motor[brdt] = c4-c3;
		}
		//half speed
		if(precision){
			motor[fldt] = motor[bldt] = (c4+c3)/2;
			motor[frdt] = motor[brdt] = (c4-c3)/2;
		}
		//switch for above
		if(vexRT[Btn8L]==1)
			precision=!precision;
		//
		if(vexRT[Btn5U])motor[mgml] = motor[mgmr] = 60;
		else if(vexRT[Btn5D])motor[mgml] = motor[mgmr] = -60;
		else motor[mgml] = motor[mgmr] = 0;
		//
		
		//sounds
		if(!bSoundActive){
			if(vexRT[Btn8R]==1){
				wait1Msec(100);
				playSoundFile("allstar.wav");
			}
			if(vexRT[Btn8U]==1){
				wait1Msec(100);
				playSoundFile("omae_wa_mou_shindeiru.wav");
			}
		}
		clearLCDLine(0);
		clearLCDLine(1);
		displayLCDString(0,0,"Battery: ");
		displayLCDNumber(0,9, nImmediateBatteryLevel);
		displayLCDString(0,13, " mV");
		displayLCDString(1,0,"Backup: ");
		displayLCDNumber(1,9,BackupBatteryLevel);
		displayLCDString(1,13, " mV");
		wait1Msec(25);
	}
}

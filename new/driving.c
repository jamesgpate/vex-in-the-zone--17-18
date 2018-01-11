#include "lights.c"
#include "auton.c"
float potKp = .5; 
float potKi = .5; 
float potKd = .5;
float dr4bKp = 75; 
float dr4bKi = 0; 
float dr4bKd = 0;
task drive(){
	bool precision = false;
	int c2 = 0;
	int c3 = 0;
	SensorValue[towerL] = 0;
	SensorValue[towerR] = 0;
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
			motor[port4] = motor[port6] = c3;
			motor[port5] = motor[port7] =  c2;
		}
		//half speed
		if(precision){
			motor[port4] = motor[port6] = c3/2;
			motor[port5] = motor[port7] = c2/2;
		}
		//switch for above
		if(vexRT[Btn8L]==1)
			precision=!precision;
		//6U/6D for dr4b, 5U for half speed
		if(vexRT[Btn6U]==1)
			motor[rdr4b] = 60;
		else if(vexRT[Btn6D]==1)
			motor[rdr4b] = -60;
		else
			motor[rdr4b] = 0;
		//open and close claw
		if(vexRT[Btn5U]==1)
			motor[claw] = 127;
		else if(vexRT[Btn5D]==1)
			motor[claw] = -127;
		else
			motor[claw] = 0;
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
		if(vexRT[Btn6U]==0&&vexRT[Btn6D]==0){
			motor[ldr4b]=0;
			motor[rdr4b]=0;
		}*/
		//fourbar
		motor[fourbar] = c2;
		//lights
		if(fadeColorsButton){
			stopTask(fadeColors);
			stopTask(sendRainbowDownStrip);
			startTask(fadeColors);
		}
		if(sendRainbowDownStripButton){
			stopTask(fadeColors);
			stopTask(sendRainbowDownStrip);
			startTask(sendRainbowDownStrip);
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
		wait1Msec(25);
	}
}

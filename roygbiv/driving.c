#include "lights.c"
int currentTowerAngle = 0;
int currentElbowAngle = 0;
task drive(){
	int c4 = 0, c3 = 0, c2 = 0, c1 = 0;
	resetMotorEncoder(tower);
	resetMotorEncoder(elbow);
	while(true){
		currentTowerAngle = (360*nMotorEncoder[tower])/(5*392);
		currentElbowAngle = -(360*nMotorEncoder[elbow])/(5*627.2) + currentTowerAngle;
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
		motor[fldt] = motor[bldt] = c4+c3;
		motor[frdt] = motor[brdt] = -c4+c3;
		//mobile goal
		if(vexRT[Btn5U])motor[mgml] = motor[mgmr] = 90;
		else if(vexRT[Btn5D])motor[mgml] = motor[mgmr] = -90;
		else motor[mgml] = motor[mgmr] = 0;
		//claw
		if(vexRT[Btn6U]==1)	motor[claw]=127;
		else if(vexRT[Btn6D]==1) motor[claw]=-127;
		else motor[claw]=0;
		//arm control with fancy schmancy holding technique
		motor[tower] = /*vexRT[Ch3Xmtr2]/2 + */c2 - ((.19*sinDegrees(currentTowerAngle)+.5*(.24*sinDegrees(currentElbowAngle)))/(.5+.19))*15;
		motor[elbow] = /*vexRT[Ch2Xmtr2]/2 + */c1 + ((.19*sinDegrees(currentTowerAngle)+.5*(.24*sinDegrees(currentElbowAngle)))/(.5+.19))*5;
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
		wait1Msec(25);
	}
}

//#include "lights.c"
task drive(){
	int dr4bEncAvg = (SensorValue[ldr4bEnc]+SensorValue[rdr4bEnc])/2;
	int c4 = 0, c3 = 0, c2 = 0, c1 = 0;
	float potTarget = SensorValue[fourbarPot], potError = 0, potLastError = 0, potDerivative = 0, potIntegral = 0, potKp = .5, potKi = .5, potKd = .5, potPower;
	float dr4bTarget = dr4bEncAvg, dr4bError = 0, dr4bLastError = 0, dr4bDerivative = 0, dr4bIntegral = 0, dr4bKp = 30, dr4bKi = 30, dr4bKd = 30, dr4bPower;
	while(true){
		long sysTime = nSysTime;
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
		if(!vexRT[Btn8L]){
			motor[ldt1] = motor[ldt2] = c3+(c4/2);
			motor[rdt1] = motor[rdt2] = -c3+(c4/2);
		}
		else if(vexRT[Btn8L]){
			motor[ldt1] = motor[ldt2] = c3+(c4/4);
			motor[rdt1] = motor[rdt2] = -c3+(c4/4);
		}
		//mobile goal
		if(vexRT[Btn5U])motor[mgml] = motor[mgmr] = -127;
		else if(vexRT[Btn5D])motor[mgml] = motor[mgmr] = 127;
		else motor[mgml] = motor[mgmr] = 0;
		//claw
		if(vexRT[Btn7D]||vexRT[Btn6DXmtr2])motor[claw] = -20;
		else motor[claw] =  20;
		//dr4b
		dr4bTarget += (vexRT[Btn6U]-vexRT[Btn6D]);
		if(dr4bKi != 0){
			if(abs(dr4bError) < 50)
				dr4bIntegral = dr4bIntegral + dr4bError;
			else
				dr4bIntegral = 0;
		}
		else
			dr4bIntegral = 0;
		dr4bError = dr4bTarget - dr4bEncAvg;
		dr4bDerivative = dr4bError - dr4bLastError;
		dr4bLastError = dr4bError;
		dr4bPower = (dr4bKp * dr4bError) + (dr4bKi * dr4bIntegral) + (dr4bKd * dr4bDerivative);
		motor[ldr4b] = dr4bPower;
		motor[rdr4b] = -dr4bPower;
		//fourbar
		if(vexRT[Btn8R]){
			potTarget = 1880;
		}else{
			potTarget += c2/4;
		}
		if(potKi != 0){
			if(abs(potError) < 50)
				potIntegral = potIntegral + potError;
			else
				potIntegral = 0;
		}
		else
			potIntegral = 0;
		potError = potTarget - SensorValue[fourbarPot];
		potDerivative = potError - potLastError;
		potLastError = potError;
		potPower = (potKp * potError) + (potKi * potIntegral) + (potKd * potDerivative);
		if(potPower>127)potPower = 127;
		else if (potPower<-127)potPower = -127;
		motor[fourbar] = potPower;
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
		/*if(vexRT[Btn7U]==1){
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
		}*/
		//displays current battery and backup battery voltage
		clearLCDLine(0);
		clearLCDLine(1);
		displayLCDString(0,0,"Battery: ");
		displayLCDNumber(0,9, nAvgBatteryLevel);
		displayLCDString(0,13, " mV");
		displayLCDString(1,0,"Backup: ");
		displayLCDNumber(1,9,BackupBatteryLevel);
		displayLCDString(1,13, " mV");
		//keep the loop timing consistently 25 ms
		int timeDiff = nSysTime - sysTime;
		wait1Msec(25-timeDiff);
		EndTimeSlice();
	}
}

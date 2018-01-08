#include "lights.c"
#include "auton.c"
float potKp = .5; 
float potKi = .5; 
float potKd = .5;
float dr4bKp = 75; 
float dr4bKi = 0; 
float dr4bKd = 0;
task drive(){
	int dr4bEncAvg = (SensorValue[ldr4bEnc]+SensorValue[rdr4bEnc])/2;
	int c4 = 0, c3 = 0, c2 = 0, c1 = 0;
	int mode = 0;
	float potTarget = SensorValue[potEnc], potError = 0, potLastError = 0, potDerivative = 0, potIntegral = 0, potPower;
	float dr4bTarget = dr4bEncAvg, dr4bError = 0, dr4bLastError = 0, dr4bDerivative = 0, dr4bIntegral = 0, dr4bPower;
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
		motor[ldt1] = motor[ldt2] = c3+(c4/2);
		motor[rdt1] = motor[rdt2] = -c3+(c4/2);
		//mobile goal
		if(vexRT[Btn8D])motor[mgml] = motor[mgmr] = -127;
		else if(vexRT[Btn8U])motor[mgml] = motor[mgmr] = 127;
		else motor[mgml] = motor[mgmr] = 0;
		//claw
		switch(mode){
			case 0:
				if((SensorValue[potEnc]-33)>-45&&((abs(SensorValue[ldr4bEnc])+abs(SensorValue[rdr4bEnc]))/2)<25){
						motor[claw]=127;
					if(vexRT[Btn5D]==1){
						motor[claw]=-127;
					}
				}
				else{
					motor[claw]=15;
					if(vexRT[Btn5D]==1){
						motor[claw]=-127;
					}
				}
				break;
			case 1:
				motor[ldt1]=motor[ldt2]=10;
				motor[rdt1]=motor[rdt2]=-10;
				if((SensorValue[potEnc]-33)>-70&&((abs(SensorValue[ldr4bEnc])+abs(SensorValue[rdr4bEnc]))/2)<60){
						motor[claw]=127;
					if(vexRT[Btn5D]==1){
						motor[claw]=-127;
					}
				}
				else{
					motor[claw]=15;
					if(vexRT[Btn5D]==1){
						motor[claw]=-127;
					}
				}
				break;
		}
		if(vexRT[Btn5U]){
			SensorValue[rdr4bEnc]=0;
			SensorValue[ldr4bEnc]=0;
			SensorValue[potEnc]=0;
		}
		while(vexRT[Btn7L]==1){
			int error = 10-((SensorValue[rdr4bEnc]-SensorValue[ldr4bEnc])/2);
			motor[ldr4b]=-4*error;
			motor[rdr4b]=4*error;

		}
		while(vexRT[Btn7R]==1){
			int error = 32-((SensorValue[rdr4bEnc]-SensorValue[ldr4bEnc])/2);
			motor[ldr4b]=-4*error;
			motor[rdr4b]=4*error;
		}
		while(vexRT[Btn8L]==1){
			int error = 1-((SensorValue[rdr4bEnc]-SensorValue[ldr4bEnc])/2);
			motor[ldr4b]=-4*error;
			motor[rdr4b]=4*error;

		}
		while(vexRT[Btn8R]==1){
			int error = 90-((SensorValue[rdr4bEnc]-SensorValue[ldr4bEnc])/2);
			motor[ldr4b]=-4*error;
			motor[rdr4b]=4*error;
		}
		//dr4b - PID version
		dr4bTarget += vexRT[Btn6U]-vexRT[Btn6D];
		if(dr4bTarget>95){
			dr4bTarget = 95;
		}
		else if(dr4bTarget<0){
			dr4bTarget = 0;
		}
		writeDebugStreamLine("Value:",dr4bTarget);
		dr4bError = dr4bTarget - dr4bEncAvg;
		dr4bDerivative = dr4bError - dr4bLastError;
		if(dr4bKi != 0){
			if(abs(dr4bError) < 50)
				dr4bIntegral = dr4bIntegral + dr4bError;
			else
				dr4bIntegral = 0;
		}
		else
			dr4bIntegral = 0;
		dr4bLastError = dr4bError;
		dr4bPower = (dr4bKp * dr4bError) + (dr4bKi * dr4bIntegral) + (dr4bKd * dr4bDerivative);
		motor[ldr4b] = -dr4bPower;
		motor[rdr4b] = dr4bPower;
		//dr4b - Non-PID version
		/*if(vexRT[Btn6U]){
			motor[ldr4b]=-127;
			motor[rdr4b]=127;
		}
		if(vexRT[Btn6D]){
			motor[ldr4b]=127;
			motor[rdr4b]=-127;
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

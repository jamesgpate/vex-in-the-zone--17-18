#include "auton.c"
#include "Truespeed.h"
//
task drive(){
	int dr4bEncAvg = (SensorValue[ldr4bEnc]+SensorValue[rdr4bEnc])/2;
	int c4 = 0, c3 = 0, c2 = 0, c1 = 0;
	int mode = 0;
	while(true){
		long sysTime = nSysTime;
		//Drivetrain
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
		motor[ldt1] = c3+c4;
		motor[ldt2] = -c3+c4;
		motor[rdt1] = c3+c4;
		motor[rdt2] = -c3+c4;
		//mobile goal
		if(vexRT[Btn8U])motor[mgm] = -127;
		else if(vexRT[Btn8D])motor[mgm] = 127;
		else motor[mgm] = 0;
		//claw
		if(c2>20){
			while(c2>20){
				int fourbarError = 3000-SensorValue[fourbarPot];
				motor[fourbar]=2*fourbarError;
			}
		}
		else{
				int fourbarError = 1600-SensorValue[fourbarPot];
				motor[fourbar]=2*fourbarError;
			}

		switch(mode){
			case 0:
				if((SensorValue[fourbarPot])<1800&&((abs(SensorValue[ldr4bEnc])+abs(SensorValue[rdr4bEnc]))/2)<30){
						motor[claw]=127;
					if(vexRT[Btn5D]==1){
						motor[claw]=-127;
					}																				//If the claw is close enough to the ground, activate the claw
				}
				else{
					motor[claw]=15;
					if(vexRT[Btn5D]==1){
						motor[claw]=-127;
					}
				}
				break;
			case 1:
				if((SensorValue[fourbarPot])<2500&&((abs(SensorValue[ldr4bEnc])+abs(SensorValue[rdr4bEnc]))/2)<50){
						motor[claw]=127;
					if(vexRT[Btn5D]==1){
						motor[claw]=-127;
					}																			//Extends the range so that match loads can be easily grabbed
				}
				else{
					motor[claw]=15;
					if(vexRT[Btn5D]==1){
						motor[claw]=-127;
					}
				}
				break;
		}

		//Positioning
		while(vexRT[Btn7L]){
			int error = 10-((SensorValue[rdr4bEnc]-SensorValue[ldr4bEnc])/2); //Field Height
			motor[ldr4b]=-8*error;
			motor[rdr4b]=8*error;
			mode=0;
		}
		while(vexRT[Btn7R]){
			int error = 27-((SensorValue[rdr4bEnc]-SensorValue[ldr4bEnc])/2); //Match Load Height
			motor[ldr4b]=-8*error;
			motor[rdr4b]=8*error;
			mode=1;
		}
		while(vexRT[Btn7D]){
			int error = 1-((SensorValue[rdr4bEnc]-SensorValue[ldr4bEnc])/2);//Minimum Height
			motor[ldr4b]=-8*error;
			motor[rdr4b]=8*error;
		}
		while(vexRT[Btn7U]){
			int error = 90-((SensorValue[rdr4bEnc]-SensorValue[ldr4bEnc])/2); //Maximum Height
			motor[ldr4b]=-8*error;
			motor[rdr4b]=8*error;
		}
		//Changing modes
		if(vexRT[Btn8L]) mode = 0;
		if(vexRT[Btn8R]) mode = 1;
		//Reset the sensors for testing

		if(vexRT[Btn5U]){
			SensorValue[rdr4bEnc]=0;
			SensorValue[ldr4bEnc]=0;
			//SensorValue[fourbarEnc]=0;
			SensorValue[ldtEnc]=0;
			SensorValue[rdtEnc]=0;
		}

		//dr4b - Non-PID version
		if(vexRT[Btn6U]){
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
		}
		//fourbar
		/*if(SensorValue[fourbarEnc]>0){
			if(c2>0){
				motor[fourbar]=c2;
			}
			else motor[fourbar]=0;
		}
		else{
			motor[fourbar]=c2;
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
		//keep the loop timing consistently 20 ms
		int timeDiff = nSysTime - sysTime;
		wait1Msec(20-timeDiff);
		EndTimeSlice();
	}
}

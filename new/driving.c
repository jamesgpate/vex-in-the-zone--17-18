#include "auton.c"
#include "Truespeed.h"
#include "lights.c"
//
task drive(){
	startTask(sendRainbowDownStrip);
	int dr4bEncAvg = (SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2;
	int c4 = 0, c3 = 0, c2 = 0, c1 = 0;
	int mode = 0;
	int count = 0;
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
		//truespeed
		if(c1>0) c1 = TrueSpeed[abs(c1)];
		else if(c1<0) c1 = -TrueSpeed[abs(c1)];
		if(c2>0) c2 = TrueSpeed[abs(c2)];
		else if(c2<0) c2 = -TrueSpeed[abs(c2)];
		if(c3>0) c3 = TrueSpeed[abs(c3)];
		else if(c3<0) c3 = -TrueSpeed[abs(c3)];
		if(c4>0) c4 = TrueSpeed[abs(c4)];
		else if(c4<0) c4 = -TrueSpeed[abs(c4)];
		//send these values to the motor
		motor[ldt1] = c3+c4;
		motor[ldt2] = -c3+c4;
		motor[rdt1] = c3+c4;
		motor[rdt2] = -c3+c4;
		//mobile goal
		if(vexRT[Btn8U])motor[mgm] = 127;
		else if(vexRT[Btn8D])motor[mgm] = -127;
		else motor[mgm] = 0;


		//claw
		switch(mode){
			case 0:
				if((SensorValue[fourbarPot])>1600 && ((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2)<30){
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
				if((SensorValue[fourbarPot])>1400 && ((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2)<50){
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
			int error = 10-((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2); //Field Height
			motor[ldr4b]=-8*error;
			motor[rdr4b]=8*error;
			mode=0;
		}
		while(vexRT[Btn7R]){
			int error = 27-((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2); //Match Load Height
			motor[ldr4b]=-8*error;
			motor[rdr4b]=8*error;
			mode=1;
		}
		while(vexRT[Btn7D]){
			int error = 1-((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2);//Minimum Height
			motor[ldr4b]=-8*error;
			motor[rdr4b]=8*error;
		}
		while(vexRT[Btn7U]){
			int error = 90-((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2); //Maximum Height
			motor[ldr4b]=-8*error;
			motor[rdr4b]=8*error;
		}
		//Changing modes
		if(vexRT[Btn8L]) mode = 0;
		if(vexRT[Btn8R]) mode = 1;
		//Reset the sensors for testing

		while(vexRT[Btn5U] && count==0){
			while(SensorValue[sound]<200 && vexRT[Btn5U]){
				motor[ldr4b]=-90;
				motor[rdr4b]=90;
			}

			motor[ldr4b]=0;
			motor[rdr4b]=0;

			while(SensorValue[fourbarPot]>285 && vexRT[Btn5U] && count==0){
				int fourbarError = 285-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError;
			}
			motor[ldr4b]=127;
			motor[rdr4b]=-127;
			wait1Msec(750);
			motor[ldr4b]=0;
			motor[rdr4b]=0;
			motor[claw]=-127;
			wait1Msec(250);
			motor[claw]=-20;
			wait1Msec(125);
			motor[ldr4b]=-127;
			motor[rdr4b]=127;
			wait1Msec(250);
			motor[ldr4b]=0;
			motor[rdr4b]=0;
			motor[claw]=0;

			while(SensorValue[fourbarPot]<1750 && vexRT[Btn5U]){
				int fourbarError = 1700-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError;
			}
			while(((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2)>1 && vexRT[Btn5U]){
				int error = 10-((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2);
				motor[ldr4b]=-8*error;
				motor[rdr4b]=8*error;
				motor[fourbar]=-20;
			}
			motor[fourbar]=0;
			count=1;
		}
		count=0;

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
		if(vexRT[Ch2]>20){
			int lastFourbarError=SensorValue[fourbarPot];
			while(vexRT[ch2]>20&&SensorValue[fourbarPot]>300){
				int fourbarError = 285-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError-1*(fourbarError-lastFourbarError);
				lastFourbarError=fourbarError;
			}
		}
		if(vexRT[ch2]<-20){
			int lastFourbarError=SensorValue[fourbarPot];
			while(vexRT[ch2]<-20){
				int fourbarError = 2800-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError-1*(fourbarError-lastFourbarError);
				lastFourbarError=fourbarError;
				motor[claw]=10;
			}
		}
		else{
					if(motor[rdr4b]>15){
							motor[fourbar]= 30;
					}
					if(motor[rdr4b]<-15){
							motor[fourbar]= -20;
					}
					else motor[fourbar]=0;
			}

		//motor[fourbar]=c2;
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

#include "auton.c"
#include "Truespeed.h"
#include "lights.c"
bool colors = false;
int timed=0;
//
task drive(){
	startTask(sendRainbowDownStrip);
	int dr4bEncAvg = (SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2;
	int c4 = 0, c3 = 0, c2 = 0, c1 = 0;
	int c4Partner = 0, c3Partner = 0, c2Partner = 0, c1Partner = 0;
	int mode = 0;
	int autoStackCount = 0;
	int lcdPage = 0;
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
		if(abs(vexRT[Ch4Xmtr2])>THRESHOLD) c4Partner = vexRT[Ch4Xmtr2];
		else c4Partner = 0;
		if(abs(vexRT[Ch3Xmtr2])>THRESHOLD) c3Partner = vexRT[Ch3Xmtr2];
		else c3Partner = 0;
		if(abs(vexRT[Ch2Xmtr2])>THRESHOLD) c2Partner = vexRT[Ch2Xmtr2];
		else c2Partner = 0;
		if(abs(vexRT[Ch1Xmtr2])>THRESHOLD) c1Partner = vexRT[Ch1Xmtr2];
		else c1Partner = 0;
		//truespeed
		if(c1>0) c1 = TrueSpeed[abs(c1)];
		else if(c1<0) c1 = -TrueSpeed[abs(c1)];
		if(c2>0) c2 = TrueSpeed[abs(c2)];
		else if(c2<0) c2 = -TrueSpeed[abs(c2)];
		if(c3>0) c3 = TrueSpeed[abs(c3)];
		else if(c3<0) c3 = -TrueSpeed[abs(c3)];
		if(c4>0) c4 = TrueSpeed[abs(c4)];
		else if(c4<0) c4 = -TrueSpeed[abs(c4)];
		if(c1Partner>0) c1Partner = TrueSpeed[abs(c1Partner)];
		else if(c1Partner<0) c1Partner = -TrueSpeed[abs(c1Partner)];
		if(c2Partner>0) c2Partner = TrueSpeed[abs(c2Partner)];
		else if(c2Partner<0) c2Partner = -TrueSpeed[abs(c2Partner)];
		if(c3Partner>0) c3Partner = TrueSpeed[abs(c3Partner)];
		else if(c3Partner<0) c3Partner = -TrueSpeed[abs(c3Partner)];
		if(c4Partner>0) c4Partner = TrueSpeed[abs(c4Partner)];
		else if(c4Partner<0) c4Partner = -TrueSpeed[abs(c4Partner)];
		//send these values to the motor
		motor[ldt1] = c3+c4;
		motor[ldt2] = -c3+c4;
		motor[rdt1] = c3+c4;
		motor[rdt2] = -c3+c4;
		/*
		motor[ldt1] = c3Partner;
		motor[ldt2] = -c3Partner;
		motor[rdt1] = c2Partner;
		motor[rdt2] = -c2Partner;*/
		//mobile goal
		if(vexRT[Btn8U])motor[mgm] = 127;
		else if(vexRT[Btn8D])motor[mgm] = -127;
		else motor[mgm] = 0;
		//claw
		switch(mode){
			case 0:
				if((SensorValue[fourbarPot])>1300 && ((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2)<20){
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
				if((SensorValue[fourbarPot])>1100 && ((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2)<35){
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
		while(vexRT[Btn7R]){
			int error = 27-((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2); //Match Load Height
			motor[ldr4b]=-8*error;
			motor[rdr4b]=8*error;
			mode=1;
		}
		while(vexRT[Btn7L]){
			int error = 1-((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2);//Minimum Height/Field Height
			motor[ldr4b]=-8*error;
			motor[rdr4b]=8*error;
		}
		while(vexRT[Btn7U]){
			int error = 100-((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2); //Maximum Height
			motor[ldr4b]=-8*error;
			motor[rdr4b]=8*error;
		}
		//Changing modes
		if(vexRT[Btn8L]) mode = 0;
		if(vexRT[Btn8R]) mode = 1;
		//Reset the sensors for testing

		while(vexRT[Btn5U] && autoStackCount==0){
			while(SensorValue[fourbarPot]>1525 && vexRT[Btn5U]){
				int fourbarError = 1525-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError;
			}
			while(SensorValue[sound]<230 && vexRT[Btn5U] && autoStackCount==0){
				motor[ldr4b]=-90;
				motor[rdr4b]=90;
			}

			motor[ldr4b]=0;
			motor[rdr4b]=0;

			while(SensorValue[fourbarPot]>460 && vexRT[Btn5U] && autoStackCount==0){
				int fourbarError = 460-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError;
			}
			motor[ldr4b]=127;
			motor[rdr4b]=-127;
			wait1Msec(400);
			motor[ldr4b]=0;
			motor[rdr4b]=0;
			motor[claw]=-127;
			wait1Msec(250);
			motor[claw]=-40;
			wait1Msec(125);
			motor[ldr4b]=-127;
			motor[rdr4b]=127;
			wait1Msec(250);
			motor[ldr4b]=0;
			motor[rdr4b]=0;
			motor[claw]=0;

			while(SensorValue[fourbarPot]<1525 && vexRT[Btn5U]){
				int fourbarError = 1525-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError;
			}
			while(((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2)>1 && vexRT[Btn5U]){
				int error = 1-((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2);
				motor[ldr4b]=-8*error;
				motor[rdr4b]=8*error;
				motor[fourbar]=-20;
			}
			motor[fourbar]=0;
			while(SensorValue[fourbarPot]<1800 && vexRT[Btn5U]){
				int fourbarError = 1800-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError;
			}
			autoStackCount=1;
			motor[claw]=127;
			while(vexRT[Btn5U])wait1Msec(1);
		}
		autoStackCount=0;

		//dr4b - Non-PID version
		if(vexRT[Btn6U]){
			motor[ldr4b]=-127;
			motor[rdr4b]=127;
		}
		/*if(vexRT[Btn6U] && SensorValue[sound]>250){
			motor[ldr4b]=0;
			motor[rdr4b]=0;
		}*/
		if(vexRT[Btn6U] && SensorValue[fourbarPot]<1250 && SensorValue[fourbarPot]>1500){
			while(SensorValue[fourbarPot]<1400 && vexRT[Btn6U]){
				int fourbarError = 1400-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError;
			}
		}


		if(vexRT[Btn6D]){
			motor[ldr4b]=127;
			motor[rdr4b]=-127;
		}
		if(vexRT[Btn6U]==0&&vexRT[Btn6D]==0){
			motor[ldr4b]=0;
			motor[rdr4b]=0;
		}
		if(((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])<5)&&vexRT[Btn6U]==0&&vexRT[Btn6D]==0){
			motor[ldr4b]=15;
			motor[rdr4b]=-15;
		}
		if(vexRT[Btn7D] && !colors && (nSysTime-timed)>3000){
			startTask(slowFade);
			colors = true;
			timed = nSysTime;
		}
		if(vexRT[Btn7D] && colors && (nSysTime-timed)>3000){
			stopTask(slowFade);
			colors = false;
			timed = nSysTime;
		}



		//fourbar
		if(vexRT[Ch2]>20 && SensorValue[sound]>250){
			while(vexRT[Ch2]>20&&SensorValue[fourbarPot]>460 && SensorValue[sound]>250){
				int fourbarError = 460-SensorValue[fourbarPot];
				if(fourbarError>-20)fourbarError=0;
				if(fourbarError>-40) fourbarError = fourbarError/10;
				motor[fourbar]=-.5*fourbarError;
				motor[claw]=10;
				if(vexRT[Btn5D]) motor[claw]=-127;
				
			}
		}
		if(vexRT[Ch2]>20 && SensorValue[sound]<230 && dr4bEncAvg>20){
			while(vexRT[Ch2]>20&&SensorValue[fourbarPot]>1525 && SensorValue[sound]<250){
				int fourbarError = 1525-SensorValue[fourbarPot];
				if(fourbarError>-20)fourbarError=0;
				if(fourbarError>-40) fourbarError = fourbarError/10;
				motor[fourbar]=-.5*fourbarError;
				motor[claw]=10;
				if(vexRT[Btn5D]) motor[claw]=-127;
				
			}
		}
		if(vexRT[Ch2]<-20 && dr4bEncAvg<20){
			while(vexRT[Ch2]<-20){
				int fourbarError = 1800-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError;
				motor[claw]=127;
			}
		}
		if(vexRT[Ch2]<-20 && dr4bEncAvg>20){
				while(vexRT[Ch2]<-20){
				int fourbarError = 1525-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError;
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
		switch(lcdPage){
			case 0:
				clearLCDLine(0);
				clearLCDLine(1); 
				displayLCDString(0,0,"Battery: ");
				displayLCDNumber(0,9, nAvgBatteryLevel);
				displayLCDString(0,13, " mV");
				displayLCDString(1,0,"Backup: ");
				displayLCDNumber(1,9, BackupBatteryLevel);
				displayLCDString(1,13, " mV");
				break;
			case 1:
				clearLCDLine(0);
				clearLCDLine(1); 
				displayLCDString(0,0,"Left DT: ");
				displayLCDNumber(0,10, SensorValue[ldtEnc]);
				displayLCDString(1,0,"Right DT: ");
				displayLCDNumber(1,10, SensorValue[rdtEnc]);
				break;
			case 2:
				clearLCDLine(0);
				clearLCDLine(1); 
				displayLCDString(0,0,"Left DR4B: ");
				displayLCDNumber(0,10, SensorValue[ldr4bEnc]);
				displayLCDString(1,0,"Right DR4B: ");
				displayLCDNumber(1,10, SensorValue[rdr4bEnc]);
			case 3:
				clearLCDLine(0);
				clearLCDLine(1); 
				displayLCDString(0,0,"Sonar: ");
				displayLCDNumber(0,10, SensorValue[sound]);
				displayLCDString(1,0,"4bar Pot: ");
				displayLCDNumber(1,10, SensorValue[fourbarPot]);
		}
		if(nLCDButtons)
		//keep the loop timing consistently 20 ms
		int timeDiff = nSysTime - sysTime;
		wait1Msec(20-timeDiff);
		EndTimeSlice();
	}
}

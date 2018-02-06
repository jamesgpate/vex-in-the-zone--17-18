#include "auton.c"
#include "lights.c"

bool colors = false;
int timed = 0;
int toggle = 0;

const int fourbarTop = 460;
const int fourbarParallel = 1525;
const int fourbarBottom = 2000;

const int coneDistance = 230;

const short leftButton = 1;
const short centerButton = 2;
const short rightButton = 4;
//
int autoStack(int stackCount){
	//start at parallel dr4b and parallel 4b
	//pid vars
	//run claw
	//dr4b full height
	//4b vertical
	//go down based on proportion of stack count
	//release claw
	//dr4b full height
	//run claw
	//4b horizontal
	//down to match load height
	return stackCount++;
}
task drive(){
	int dr4bEncAvg = (SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2;
	int c4 = 0, c3 = 0, c2 = 0, c1 = 0;
	int c4Partner = 0, c3Partner = 0, c2Partner = 0, c1Partner = 0;
	int clawMode = 0;
	int autoStackCount = 0;
	int lcdPage = 0;
	bool partnerControl = false;
	while(true){
		dr4bEncAvg = (SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2;
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
		//send these values to the motor
		if(!partnerControl){
			motor[ldt1] = c3+c4;
			motor[ldt2] = c3+c4;
			motor[rdt1] = -c3+c4;
			motor[rdt2] = -c3+c4;
		}
		if(partnerControl){
			motor[ldt1] = c3Partner;
			motor[ldt2] = c3Partner;
			motor[rdt1] = -c2Partner;
			motor[rdt2] = -c2Partner;
		}
		//mobile goal
		if(partnerControl){
		if(vexRT[Btn7DXmtr2])motor[mgm] = 127;
		else if(vexRT[Btn7UXmtr2])motor[mgm] = -127;
		else motor[mgm] = 0;
		}
		if(!partnerControl){
		if(vexRT[Btn8U])motor[mgm] = 127;
		else if(vexRT[Btn8D])motor[mgm] = -127;
		else motor[mgm] = 0;
		}

		//claw
		switch(clawMode){
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
			clawMode=1;
		}
		while(vexRT[Btn7L]){
			int error = 1-((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2);//Minimum Height/Field Height
			motor[ldr4b]=-8*error;
			motor[rdr4b]=8*error;
		}
		//Changing clawModes
		if(vexRT[Btn8L]) clawMode = 0;
		if(vexRT[Btn8R]) clawMode = 1;

		//Auto Stacking
		while(vexRT[Btn5U] && autoStackCount==0){

			while(SensorValue[fourbarPot]>fourbarParallel && vexRT[Btn5U]){
				int fourbarError = fourbarParallel-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError;
			}

			while(SensorValue[sound]<coneDistance && vexRT[Btn5U] && autoStackCount==0){
				motor[ldr4b]=-90;
				motor[rdr4b]=90;
			}
			wait1Msec(250);
			motor[ldr4b]=0;
			motor[rdr4b]=0;

			while(SensorValue[fourbarPot]>fourbarTop && vexRT[Btn5U] && autoStackCount==0){
				int fourbarError = fourbarTop-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError;
			}
			wait1Msec(100);
			motor[ldr4b]=127;
			motor[rdr4b]=-127;
			wait1Msec(350);
			motor[ldr4b]=0;
			motor[rdr4b]=0;
			motor[claw]=-127;
			wait1Msec(250);
			motor[claw]=-80;
			wait1Msec(125);
			motor[ldr4b]=-127;
			motor[rdr4b]=127;
			wait1Msec(150);
			motor[ldr4b]=0;
			motor[rdr4b]=0;
			motor[claw]=0;

			while(SensorValue[fourbarPot]<fourbarParallel && vexRT[Btn5U]){
				int fourbarError = fourbarParallel-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError;
			}

			while(((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2)>1 && vexRT[Btn5U]){
				int error = 1+(clawMode*19)-((SensorValue[ldr4bEnc]-SensorValue[rdr4bEnc])/2);
				motor[ldr4b]=-8*error;
				motor[rdr4b]=8*error;
				motor[fourbar]=20;
			}
			motor[fourbar]=0;
			while(SensorValue[fourbarPot]<fourbarBottom && vexRT[Btn5U]){
				int fourbarError = fourbarBottom-SensorValue[fourbarPot];
				motor[fourbar]=-2*fourbarError;
				motor[ldr4b]=15;
				motor[rdr4b]=-15;

			}
			autoStackCount=1;
			motor[claw]=127;
			motor[ldr4b]=0;
			motor[rdr4b]=0;
			while(vexRT[Btn5U])wait1Msec(1);
		}
		autoStackCount=0;

		//dr4b - Non-PID version
		if(vexRT[Btn6U]){
			while(vexRT[Btn6U] && dr4bEncAvg<90 && SensorValue[fourbarPot]>1525){
			int fourbarError = fourbarParallel-SensorValue[fourbarPot];
			motor[fourbar]=-0.5*fourbarError;
			motor[ldr4b]=-127;
			motor[rdr4b]=127;
			}
		}else if(vexRT[Btn6D]){
			while(vexRT[Btn6D] && dr4bEncAvg>5 && SensorValue[fourbarPot]<1525){
			int fourbarError = fourbarParallel-SensorValue[fourbarPot];
			motor[fourbar]=-0.5*fourbarError;
			motor[ldr4b]=127;
			motor[rdr4b]=-127;
			}
		}else if(!vexRT[Btn6U] && !vexRT[Btn6D]){
			motor[ldr4b]=0;
			motor[rdr4b]=0;
		}
		if((dr4bEncAvg<5)&&vexRT[Btn6U]==0&&vexRT[Btn6D]==0){
			motor[ldr4b]=15;
			motor[rdr4b]=-15;
		}
		//LEDS
		if(vexRT[Btn7D] && !colors && (nSysTime-timed)>3000 && !partnerControl){
			startTask(slowFade);
			colors = true;
			timed = nSysTime;
		}
		if(vexRT[Btn7D] && colors && (nSysTime-timed)>3000 && !partnerControl){
			stopTask(slowFade);
			colors = false;
			timed = nSysTime;
		}
		//fourbar
		if(vexRT[Ch2]>20){
			while(vexRT[Ch2]>20&&SensorValue[fourbarPot]>fourbarTop){
				int fourbarError = fourbarTop-SensorValue[fourbarPot];
				if(fourbarError>-20)fourbarError=0;
				if(fourbarError>-40) fourbarError = fourbarError/10;
				motor[fourbar]=-.5*fourbarError;
				motor[claw]=10;
				if(vexRT[Btn5D]) motor[claw]=-127;

			}
		}
		if(vexRT[Ch2]<-20 && dr4bEncAvg<10){
			while(vexRT[Ch2]<-20){
				int fourbarError = fourbarBottom-SensorValue[fourbarPot];
				motor[fourbar]=-1.5*fourbarError;
				motor[claw]=127;
			}
		}
		if(vexRT[Ch2]<-20 && dr4bEncAvg>10){
				while(vexRT[Ch2]<-20){
				int fourbarError = fourbarParallel-SensorValue[fourbarPot];
				motor[fourbar]=-1.5*fourbarError;
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
		if(nLCDButtons == leftButton){
			wait1Msec(50);
			if(lcdPage == 0) lcdPage = 3;
			else lcdPage--;
		}else if(nLCDButtons == rightButton){
			wait1Msec(50);
			if(lcdPage == 3) lcdPage = 0;
			else lcdPage++;
		}
		if(nLCDButtons == centerButton && toggle>3000) {
			wait1Msec(50);
			partnerControl = !partnerControl;
			toggle = nSysTime;
		}
		//keep the loop timing consistently 20 ms
		int timeDiff = nSysTime - sysTime;
		wait1Msec(20-timeDiff);
		EndTimeSlice();
	}
}

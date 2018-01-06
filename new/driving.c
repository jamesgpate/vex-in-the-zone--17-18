//#include "lights.c"
#include "auton.c"
task drive(){
	int dr4bEncAvg = (SensorValue[ldr4bEnc]+SensorValue[rdr4bEnc])/2;
	int c4 = 0, c3 = 0, c2 = 0, c1 = 0;
	int mode = 0;
	float potTarget = SensorValue[potEnc], potError = 0, potLastError = 0, potDerivative = 0, potIntegral = 0, potKp = .5, potKi = .5, potKd = .5, potPower;
	float dr4bTarget = dr4bEncAvg, dr4bError = 0, dr4bLastError = 0, dr4bDerivative = 0, dr4bIntegral = 0, dr4bKp = 75, dr4bKi = 0, dr4bKd = 0, dr4bPower;
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
				if((SensorValue[potEnc]-33)>-55&&((abs(SensorValue[ldr4bEnc])+abs(SensorValue[rdr4bEnc]))/2)<60){
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
			case 2:
			if(vexRT[Btn5D])motor[claw]=-127;
			if(vexRT[Btn5U])motor[claw]=127;
			if(!(vexRT[btn5D])&&!(vexRT[Btn5U]))motor[claw]=-10;
			break;
	}
	if(vexRT[Btn7D])mode=0;
	if(vexRT[Btn7U])mode=1;
	if(vexRT[Btn7L])mode=2;
	if(vexRT[Btn7R])mode=3;

	if(mode==3){																			//X=height of cone Y=radius of 4b Z=height of mg
		for(int i=0;i<12;i++){
			rotateDr4bUpTo((12+C_fourbarRadius+C_coneHeight*i);
			rotateFourbarTo(90); //change
			harvesterUp();
			rotateFourbarTo(180); //change
			rotateDr4bDownTo((12+C_fourbarRadius-(C_coneHeight*i);
			harvesterDown();
		}
	}




		//dr4b
/*		if(vexRT[Btn6U]==1){													//Start PID
			dr4bTarget++;
		}
		if(vexRT[Btn6D]==1){
			dr4bTarget--;
		}
		if(dr4bTarget>95){
			dr4bTarget=95;
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
	//	motor[ldr4b] = -dr4bPower;
	//	motor[rdr4b] = dr4bPower;
		wait1Msec(35); */					//End PID
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
		motor[fourbar] = c2;
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

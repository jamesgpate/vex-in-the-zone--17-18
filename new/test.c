#pragma config(Sensor, in1,    gyro,           sensorGyro)
#include "gyroLib.c"
task main(){
	delay(1200);
	GyroInit(1);
	while(true){
		string gyroStr = GyroAngleDegGet();
		writeDebugStream(gyroStr);
		motor[port1]=vexRT[Ch2];
		motor[port2]=vexRT[Ch2];
		motor[port3]=vexRT[Ch2];
		motor[port4]=vexRT[Ch2];
		motor[port5]=vexRT[Ch2];
		motor[port6]=vexRT[Ch2];
		motor[port7]=vexRT[Ch2];
		motor[port8]=vexRT[Ch2];
		motor[port9]=vexRT[Ch2];
		motor[port10]=vexRT[Ch2];
	}
}

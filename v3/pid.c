#include "main.c"
float pid_Kp;
float pid_Ki;
float pid_Kd;
static int pidRunning = 1;
void pid(tMotor motorNum, tSensors sensor, float kp, float ki, float kd, float pidRequestedValue){
	float pidSensorCurrentValue;
	float pidError;
	float pidLastError;
	float pidIntegral;
	float pidDerivative;
	float pidDrive;
	pidLastError = 0;
	pidIntegral = 0;
	pid_Kp = kp;
	pid_Ki = ki;
	pid_Kd = kd;
	if(pidRunning){
		pidSensorCurrentValue = SensorValue[sensor];
		pidError = pidSensorCurrentValue - pidRequestedValue;
		if(pid_Ki!=0){
			if(abs(pidError)<50){
				pidIntegral = pidIntegral + pidError;
			}else{
				pidIntegral = 0;
			}
		}else{
			pidIntegral = 0;
		}
		pidDerivative = pidError - pidLastError;
		pidLastError = pidError;
		pidDrive = (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);
		if(pidDrive > 127) pidDrive = 127;
		if(pidDrive < -127) pidDrive = -127;
		motor[motorNum] = pidDrive;
	}else{
		pidError = 0;
		pidLastError = 0;
		pidIntegral = 0;
		pidDerivative = 0;
		motor[motorNum] = 0;
	}
}

/*
    Copyright (C) 2017 Quantum Robotics

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
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
		SensorValue[sensor] = 0;
	}else{
		pidError = 0;
		pidLastError = 0;
		pidIntegral = 0;
		pidDerivative = 0;
		motor[motorNum] = 0;
	}
}

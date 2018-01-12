#include "main.c"
const string enterString = "<     Enter    >";
const string firstAutonString = "Left Side MGM";
const string secondAutonString = "Right Side MGM";
const string thirdAutonString = "Luke";
const string fourthAutonString = "Test";
int lcdCount = 0;
const int C_rOfWheels = 2;
const int C_rOfRobot = 13;
const float C_PI = 3.1415926;
const int C_motorPower = 70;
const float C_dr4bconstant = 2.714;
//
struct gyroConfig{
	float stdDev;
	float avg;
	float voltsPerDPS;
	char gyroFlipped;
};
typedef struct {
	struct gyroConfig config;
	int portNum;
} Gyro;
#define GYRO_STD_DEVS 3
#define GYRO_OVERSAMPLE 1
#define GYRO_CALIBRATION_POINTS 2000
float calibrationBuffer [GYRO_CALIBRATION_POINTS];
float gyroGetRate (Gyro gyro);
void gyroCalibrate (Gyro gyro){
	float rawAverage = 0.0;
	float stdDev = 0.0;
	//calculate average gyro reading with no motion
	for(int i = 0; i < GYRO_CALIBRATION_POINTS; ++i){
		float raw = SensorValue (gyro.portNum);
		rawAverage += raw;
		calibrationBuffer [i] = raw;
		delay (1);
	}
	rawAverage /= GYRO_CALIBRATION_POINTS;
	gyro.config.avg = rawAverage;
	//calcuate the standard devation, or the average distance
	//from the average on the data read
	for (int i = 0; i < GYRO_CALIBRATION_POINTS; ++i)
		stdDev += fabs (rawAverage - calibrationBuffer [i]);
	stdDev /= (float) GYRO_CALIBRATION_POINTS;
	gyro.config.stdDev = stdDev;
	gyro.config.voltsPerDPS = 0.0011 * 1.515;
}
void gyroInit (Gyro gyro, int portNum, char gyroFlipped) {
	gyro.portNum = portNum;
	gyro.config.gyroFlipped = gyroFlipped;
	gyroCalibrate (gyro);
}
float gyroGetRate(Gyro gyro){
	float gyroRead = 0.0;
	#if defined (GYRO_OVERSAMPLE)
		if (GYRO_OVERSAMPLE > 0) {
			int sampleSum = 0;
			int nSamples = pow (4, GYRO_OVERSAMPLE);

			for (int i = 0; i < nSamples; ++i)
				sampleSum += SensorValue(gyro.portNum);
			gyroRead = (float) sampleSum / (float) nSamples;
		}
		else
			gyroRead = SensorValue (gyro.portNum);
	#else
		gyroRead = SensorValue (gyro.portNum);
	#endif
	float gyroDiff = gyroRead - gyro.config.avg;
	float gyroVoltage = gyroDiff * 5.0 / 4095.0;
	if (fabs (gyroDiff) > GYRO_STD_DEVS * gyro.config.stdDev)
		if (gyro.config.gyroFlipped)
			return -1 * gyroVoltage / gyro.config.voltsPerDPS;
		else
			return gyroVoltage / gyro.config.voltsPerDPS;
	return 0;
}
Gyro gyro1, gyro2;

int getEncValForDistance(int inches){//this returns the encoder value for drivetrain distance
	return (360*inches)/(C_rOfWheels*C_PI*2);
}
int getEncValForTurn(int degrees){//this returns how many times an encoder on the drivetrain needs to turn in relation to how far the robot needs to turn
	return degrees*C_rOfRobot/C_rOfWheels;
}
void moveForwards(int distance){//this moves the robot forwards *distance* inches
	int encVal = getEncValForDistance(distance);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	while(SensorValue[ldtEnc]<encVal || SensorValue[rdtEnc]>-encVal){
		motor[ldt1] = motor[ldt2] = C_motorPower;
		motor[rdt1] = motor[rdt2] = -C_motorPower;
	}
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void moveBackwards(int distance){//this moves the robot backwards *distance* inches
	int encVal = getEncValForDistance(distance);
	SensorValue[ldtEnc] = 0;
	SensorValue[rdtEnc] = 0;
	while(SensorValue[ldtEnc]>-encVal || SensorValue[rdtEnc]<encVal){
		motor[ldt1] = motor[ldt2] = -C_motorPower;
		motor[rdt1] = motor[rdt2] = C_motorPower;
	}
	motor[ldt1] = motor[ldt2] = 0;
	motor[rdt1] = motor[rdt2] = 0;
}
void gyroTurn(float fTarget){
	bool bAtGyro = false;
	long liAtTargetTime = nPgmTime;
	long liTimer = nPgmTime;
	float fGyroAngle = 0;
	while(!bAtGyro){
		//Calculate the delta time from the last iteration of the loop
		float fDeltaTime = (float)(nPgmTime - liTimer)/1000.0;
		//Reset loop timer
		liTimer = nPgmTime;

		fGyroAngle += gyroGetRate(gyro1) * fDeltaTime;
		int motorFlip;
		if(fTarget>0){
			motorFlip = 1;
		}
		else if(fTarget>0){
			motorFlip = -1;
		}
		motor[ldt1] = motor[ldt2] = C_motorPower*motorFlip;
		motor[rdt1] = motor[rdt2] = C_motorPower*motorFlip;

		//Stop the turn function when the angle has been within 3 degrees of the desired angle for 350ms
		if(abs(fTarget - fGyroAngle) > 3)
			liAtTargetTime = nPgmTime;
		if(nPgmTime - liAtTargetTime > 350){
			bAtGyro = true;
			motor[ldt1] = motor[ldt2] = 0;
			motor[rdt1] = motor[rdt2] = 0;
		}
	}
}
void turnRight(int degrees){//this turns the robot to the right *degrees* degrees
	gyroTurn((float)degrees);
}
void turnLeft(int degrees){//this turns the robot to the left *degrees* degrees
	gyroTurn((float)(-degrees));
}
void lowerMGM(){//lowers mgm
	motor[mgml] = motor[mgmr] = C_motorPower;
	wait1Msec(500);
	motor[mgml] = motor[mgmr] = 0;
}
void raiseMGM(){//raises mgm
	motor[mgml] = motor[mgmr] = -C_motorPower;
	wait1Msec(500);
	motor[mgml] = motor[mgmr] = 0;
}
void rotateDr4bUpTo(int distance){//rotates double reverse fourbar up to *distance* height
	int encVal = abs(distance*C_dr4bconstant);
	while(SensorValue[ldr4bEnc]>-encVal || SensorValue[rdr4bEnc]<encVal){
		motor[ldr4b] = -C_motorPower;
		motor[rdr4b] = C_motorPower;
		encVal = abs(distance*C_dr4bconstant);
	}
	motor[ldr4b] = motor[rdr4b] = 0;
}
void rotateDr4bDownTo(int distance){//rotates double reverse fourbar down to *distance* height
	int encVal = abs(distance*C_dr4bconstant);
	while(SensorValue[ldr4bEnc]<encVal || SensorValue[rdr4bEnc]>-encVal){
		motor[ldr4b] = C_motorPower;
		motor[rdr4b] = -C_motorPower;
		encVal = abs(distance*C_dr4bconstant);
	}
	motor[ldr4b] = motor[rdr4b] = 0;
}
void harvesterUp(){
	motor[claw] = -C_motorPower;
	wait1Msec(500);
	motor[claw] = 0;
}
void harvesterDown(){
	motor[claw] = C_motorPower;
	wait1Msec(500);
	motor[claw] = 0;
}
void rotateFourbarTo(int degrees){
	degrees = -degrees;
	int fourbarDegValue = (SensorValue[fourbarEnc]%360)*360; //correct
	if(degrees<fourbarDegValue){
		while(degrees<fourbarDegValue){
			motor[fourbar] = C_motorPower;
			fourbarDegValue = (SensorValue[fourbarEnc]%360)*360;
		}
	}else if(degrees>fourbarDegValue){
		while(degrees>fourbarDegValue){
			motor[fourbar] = -C_motorPower;
			fourbarDegValue = (SensorValue[fourbarEnc]%360)*360;
		}
	}
	motor[fourbar] = 0;
}
void robotInit(){
	motor[claw]=20;
	rotateFourbarTo(33);
}
task auton(){//main task
	getEncValForTurn(1);
	switch(lcdCount){
		case 0://first auton
			displayLCDString(0,0, firstAutonString);
			displayLCDString(1,0, "is running!");
			robotInit();
			moveForwards(40);
			harvesterUp();
			rotateDr4bUpTo(30);
			lowerMGM();
			moveForwards(8);
			harvesterDown();
			raiseMGM();
			moveBackwards(8);
			turnRight(180);
			moveForwards(36);
			turnLeft(90);
			moveForwards(18);
			turnRight(90);
			moveForwards(16);
			turnRight(45);
			moveForwards(36);
			lowerMGM();
			moveBackwards(8);
			raiseMGM();
			moveBackwards(8);
			break;
		case 1:
			displayLCDString(0,0, secondAutonString);
			displayLCDString(1,0, "is running!");
			robotInit();
			moveForwards(40);
			harvesterUp();
			rotateDr4bUpTo(30);
			lowerMGM();
			moveForwards(8);
			harvesterDown();
			raiseMGM();
			moveBackwards(8);
			turnLeft(180);
			moveForwards(36);
			turnRight(90);
			moveForwards(18);
			turnLeft(90);
			moveForwards(16);
			turnLeft(45);
			moveForwards(36);
			lowerMGM();
			moveBackwards(8);
			raiseMGM();
			moveBackwards(8);
			break;
		case 2:
			displayLCDString(0,0, thirdAutonString);
			displayLCDString(1,0, "is running!");
			robotInit();
			rotateFourbarTo(85);
			moveForwards(48);
			lowerMGM();
			moveForwards(20);
			raiseMGM();
			harvesterDown();
			moveBackwards(36);
			turnRight(45);
			moveForwards(6);
			turnLeft(45);
			moveForwards(6);
			rotateFourbarTo(33);
			moveForwards(20);
			harvesterUp();
			rotateFourbarTo(85);
			harvesterDown();
			moveForwards(4);
			rotateFourbarTo(33);
			harvesterUp();
			rotateDr4bUpTo(45);
			rotateFourbarTo(85);
			harvesterDown();
			rotateFourbarTo(33);
			rotateDr4bDownTo(1);
			moveForwards(4);
			harvesterUp();
			rotateDr4bUpTo(45);
			rotateFourbarTo(85);
			harvesterDown();
			turnLeft(180);
			moveForwards(60);
			turnRight(45);
			moveForwards(20);
			lowerMGM();
			moveBackwards(5);
			raiseMGM();
			moveBackwards(15);
			break;
		case 3:
			displayLCDString(0,0, fourthAutonString);
			displayLCDString(1,0, "is running!");
			turnRight(90);
			turnLeft(90);
			break;
		default:
			break;
	}
}

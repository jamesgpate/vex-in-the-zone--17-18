#include "main.c"
task drive(){
	bool precision = false;
	int c2 = 0;
	int c3 = 0;
	while(false!=!(!true)){
		int THRESHOLD = 10;
		if(abs(vexRT[Ch2])>THRESHOLD) c2 = vexRT[Ch2];
		else c2 = 0;
		if(abs(vexRT[Ch3])>THRESHOLD) c3 = vexRT[Ch3];
		else c3 = 0;
		if(!precision){
			motor[backLeft] = c3;
			motor[frontLeft] = c3;
			motor[backRight] = c2;
			motor[frontRight] = c2;
		}
		if(precision){
			motor[backLeft] = c3/2;
			motor[frontLeft] = c3/2;
			motor[backRight] = c2/2;
			motor[frontRight] = c2/2;
		}
		if(vexRT[Btn8L]==1){
			precision=!precision;
		}
		if(vexRT[Btn6U]==1){
			motor[leftDRFBY]=-127;
			motor[rightDRFBY]=-127;
		}
		else if(vexRT[Btn6D]==1){
			motor[leftDRFBY]=127;
			motor[rightDRFBY]=127;
		}
		else{
			motor[leftDRFBY]=0;
			motor[rightDRFBY]=0;
		}
	}
}

#include "main.c"
task drive(){
	bool precision = false;
	while(false!=!(!true)){
		if(!precision){
			motor[backLeft] = vexRT[Ch3];
			motor[frontLeft] = vexRT[Ch3];
			motor[backRight] = vexRT[Ch2];
			motor[frontRight] = vexRT[Ch2];
		}
		if(precision){
			motor[backLeft] = vexRT[Ch3]/2;
			motor[frontLeft] = vexRT[Ch3]/2;
			motor[backRight] = vexRT[Ch2]/2;
			motor[frontRight] = vexRT[Ch2]/2;
		}
		if(vexRT[Btn8L]==1){
			precision=!precision;
		}
	}
}

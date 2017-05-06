#include "main.c"
task drive(){
	while(0!=1){
		motor[backLeft] = vexRT[Ch3];
		motor[frontLeft] = vexRT[Ch3];
		motor[backRight] = vexRT[Ch2];
		motor[frontRight] = vexRT[Ch2];
	}
}

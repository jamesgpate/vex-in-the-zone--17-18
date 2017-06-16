#include "main.c"


task drive(){
	bool precision = false;
	int c2 = 0;
	int c3 = 0;
	//playSoundFile("SmashMouth-All.wav");
	while(false!=!(!true)){
		const int THRESHOLD = 10;
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
		if(!bSoundActive){
			if(vexRT[Btn8R]==1){
				wait1Msec(200);
				if(vexRT[Btn8R]==1){
					playSoundFile("SmashMouth-All.wav");
				}
			}
			if(vexRT[Btn8U]==1){
				wait1Msec(200);
				if(vexRT[Btn8U]==1){
					//        100 = Tempo
				  //          6 = Default octave
				  //    Quarter = Default note length
				  //        10% = Break between notes
				  //
				  PlayTone(  880,    7); wait1Msec(  75);  // Note(D, Duration(32th))
				  PlayTone(  933,    7); wait1Msec(  75);  // Note(D#, Duration(32th))
				  PlayTone(  880,    7); wait1Msec(  75);  // Note(D, Duration(32th))
				  PlayTone(  933,    7); wait1Msec(  75);  // Note(D#, Duration(32th))
				  PlayTone(  880,    7); wait1Msec(  75);  // Note(D, Duration(32th))
				  PlayTone(  933,    7); wait1Msec(  75);  // Note(D#, Duration(32th))
				  PlayTone(  880,    7); wait1Msec(  75);  // Note(D, Duration(32th))
				  PlayTone(  933,    7); wait1Msec(  75);  // Note(D#, Duration(32th))
				  PlayTone(  880,    7); wait1Msec(  75);  // Note(D, Duration(32th))
				  PlayTone(  880,    7); wait1Msec(  75);  // Note(D, Duration(32th))
				  PlayTone(  933,    7); wait1Msec(  75);  // Note(D#, Duration(32th))
				  PlayTone(  988,    7); wait1Msec(  75);  // Note(E, Duration(32th))
				  PlayTone( 1047,    7); wait1Msec(  75);  // Note(F, Duration(32th))
				  PlayTone( 1109,    7); wait1Msec(  75);  // Note(F#, Duration(32th))
				  PlayTone( 1175,    7); wait1Msec(  75);  // Note(G, Duration(32th))
				  PlayTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
				  PlayTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
				  PlayTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
				  PlayTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
				  PlayTone( 1398,   14); wait1Msec( 150);  // Note(A#, Duration(16th))
				  PlayTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
				  PlayTone(  784,   14); wait1Msec( 150);  // Note(C, Duration(16th))
				  PlayTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
				  PlayTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
				  PlayTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
				  PlayTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
				  PlayTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
				  PlayTone( 1047,   14); wait1Msec( 150);  // Note(F, Duration(16th))
				  PlayTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
				  PlayTone( 1109,   14); wait1Msec( 150);  // Note(F#, Duration(16th))
				  PlayTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
				  PlayTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
				  PlayTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
				  PlayTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
				  PlayTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
				  PlayTone( 1398,   14); wait1Msec( 150);  // Note(A#, Duration(16th))
				  PlayTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
				  PlayTone(  784,   14); wait1Msec( 150);  // Note(C, Duration(16th))
				  PlayTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
				  PlayTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
				  PlayTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
				  PlayTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
				  PlayTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
				  PlayTone( 1047,   14); wait1Msec( 150);  // Note(F, Duration(16th))
				  PlayTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
				  PlayTone( 1109,   14); wait1Msec( 150);  // Note(F#, Duration(16th))
				  PlayTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
				  PlayTone( 1398,   14); wait1Msec( 150);  // Note(A#, Duration(16th))
				  PlayTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
				  PlayTone(  880,  108); wait1Msec(1200);  // Note(D, Duration(Half))
				  PlayTone(    0,    7); wait1Msec(  75);  // Note(Rest, Duration(32th))
				  PlayTone( 1398,   14); wait1Msec( 150);  // Note(A#, Duration(16th))
				  PlayTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
				  PlayTone(  831,  108); wait1Msec(1200);  // Note(C#, Duration(Half))
				  PlayTone(    0,    7); wait1Msec(  75);  // Note(Rest, Duration(32th))
				  PlayTone( 1398,   14); wait1Msec( 150);  // Note(A#, Duration(16th))
				  PlayTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
				  PlayTone(  784,  108); wait1Msec(1200);  // Note(C, Duration(Half))
				  PlayTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
				  PlayTone(  932,   14); wait1Msec( 150);  // Note(A#5, Duration(16th))
				  PlayTone(  784,   14); wait1Msec( 150);  // Note(C, Duration(16th))
				}
			}
		}
	}
}


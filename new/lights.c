//word sendRainbowDownStripButton = vexRT[Btn7D];
//word fadeColorsButton = vexRT[Btn7U];
//Send 8 bits, most significant to least significant.
void sendByte(int byte){
	for(int i = 128; i >= 1; i = i / 2){
		SensorValue[data] = byte & i;
		SensorValue[clock] = 1;
		SensorValue[clock] = 0;
	}
}
//Send Start Frame
void startTransmission(){
	for(int i = 0; i < 4; i++){
		sendByte(0);
	}
}
//Send End Frame
void endTransmission(){
	for(int i = 0; i < 4; i++){
		sendByte(255);
	}
}
//Send a single LED frame, these get pushed down the strip as more and more are added
void sendLEDFrame(int brightness, int r, int g, int b){
	sendByte(224 | brightness); //0x111-----, can be 0 to 31
	sendByte(b);
	sendByte(g);
	sendByte(r);
}
//Sets the whole strip's color and brightness
void setStripColor(int length, int brightness, int r, int g, int b){
	startTransmission();
	for(int i = 0; i < length; i++){
		sendLEDFrame(brightness, r, g, b);
	}
	endTransmission();
}
void slowChange(int length, int brightness, int r, int g, int b){
	startTransmission();
	for(int i = 0; i < length; i++){
		sendLEDFrame(brightness, r, g, b);
		wait1Msec(25);
	}
	endTransmission();
}
task slowFade(){
	slowChange(120, 31, 255, 0, 0);
	slowChange(120, 31, 255, 127, 0);
	slowChange(120, 31, 255, 255, 0);
	slowChange(120, 31, 127, 255, 0);
	slowChange(120, 31, 0, 255, 0);
	slowChange(120, 31, 0, 255, 127);
	slowChange(120, 31, 0, 255, 255);
	slowChange(120, 31, 0, 127, 255);
	slowChange(120, 31, 0, 0, 255);
	slowChange(120, 31, 127, 0, 255);
	slowChange(120, 31, 255, 0, 255);
	slowChange(120, 31, 255, 0, 127);
	startTask(slowFade);
}
task purpleWave(){
	while(true){
		int x = 0;
		int brightness=31*sinDegrees(x);
		setStripColor(120, brightness,148, 0, 211);
		x++;
		wait1Msec(25);
	}
}

//Roy G Biv fade down strip
task fadeColors(){
	int r = 255;
	int g = 0;
	int b = 0;
	startTransmission();
	for(int i=0; i < 255; i++){
		sendLEDFrame(31, r, g, b);
		g++;
	}
	endTransmission();
	startTransmission();
	for(int i=0; i < 255; i++){
		sendLEDFrame(31, r, g, b);
		r--;
	}
	endTransmission();
	startTransmission();
	for(int i=0; i < 255; i++){
		sendLEDFrame(31, r, g, b);
		b++;
	}
	endTransmission();
	startTransmission();
	for(int i=0; i < 255; i++){
		sendLEDFrame(31, r, g, b);
		g--;
	}
	endTransmission();
	startTransmission();
	for(int i=0; i < 255; i++){
		sendLEDFrame(31, r, g, b);
		r++;
	}
	endTransmission();
	startTransmission();
	for(int i=0; i < 255; i++){
		sendLEDFrame(31, r, g, b);
		b--;
	}
	endTransmission();
	/*if(!fadeColorsButton){
		wait1Msec(100);
		if(!fadeColorsButton){
			startTask(fadeColors);
		}
	}*/
}
task sendRainbowDownStrip(){
	setStripColor(120,31,255,255,255);
	startTransmission();
	sendLEDFrame(31,255,0,0);
	sendLEDFrame(31,255,128,0);
	sendLEDFrame(31,255,255,0);
	sendLEDFrame(31,0,255,0);
	sendLEDFrame(31,0,0,255);
	sendLEDFrame(31,255,0,255);
	endTransmission();
	for(int i = 0; i < 114; ++i){
		startTransmission();
		sendLEDFrame(31,255,255,255);
		endTransmission();
	}
	/*if(!sendRainbowDownStripButton){
		wait1Msec(100);
		if(!sendRainbowDownStripButton){
			startTask(sendRainbowDownStrip);
		}
	}*/
}

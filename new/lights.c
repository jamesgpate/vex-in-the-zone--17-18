
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
	for(int i=0; i < 255; i++){
		sendLEDFrame(31, r, g, b);
		r--;
	}
	for(int i=0; i < 255; i++){
		sendLEDFrame(31, r, g, b);
		b++;
	}
	for(int i=0; i < 255; i++){
		sendLEDFrame(31, r, g, b);
		g--;
	}
	for(int i=0; i < 255; i++){
		sendLEDFrame(31, r, g, b);
		r++;
	}
	for(int i=0; i < 255; i++){
		sendLEDFrame(31, r, g, b);
		b--;
	}
	endTransmission();
}
//smooth rainbow with all 6 colors in succession
task smoothWave(){
	int r = 255;
	int g = 0;
	int b = 0;
	startTransmission();
	for(g = 0; g<128; g++){
		sendLEDFrame(31,r,g,b);
	}
	for(g = 128; g<255; g++){
		sendLEDFrame(31,r,g,b);
	}
	for(r = 255; r>0; r--){
		sendLEDFrame(31,r,g,b);
	}
	for(b = 0; b < 255; b++){
		g = 255 - b;
		sendLEDFrame(31,r,g,b);
	}
	for(r = 0; r < 128; r++){
		sendLEDFrame(31,r,g,b);
	}
	for(r = 128; r < 255; r++){
		b = 255-2*(r-128);
		sendLEDFrame(31,r,g,b);
	}
	endTransmission();
}
//almost same as above, but with whole LED strip
task smoothWaveFullStrip(){
	int r = 255;
	int g = 0;
	int b = 0;
	startTransmission();
	for(g = 0; g<128; g++){
		setStripColor(120,31,r,g,b);
	}
	for(g = 128; g<255; g++){
		setStripColor(120,31,r,g,b);
	}
	for(r = 255; r>0; r--){
		setStripColor(120,31,r,g,b);
	}
	for(b = 0; b < 255; b++){
		g = 255 - b;
		setStripColor(120,31,r,g,b);
	}
	for(r = 0; r < 128; r++){
		setStripColor(120,31,r,g,b);
	}
	for(r = 128; r < 255; r++){
		b = 255-2*(r-128);
		setStripColor(120,31,r,g,b);
	}
	endTransmission();
}

//red and green pulses down strip
task christmasLights(){
	startTransmission();
	playSoundFile("Jingle_Bells_7.wav");
	for(int i = 0; i < 31; i++){
		sendLEDFrame(i, 255, 0, 0);
		sendLEDFrame(i, 0, 255, 0);
	}
	endTransmission();
}
task stopLightTasks(){
	stopTask(smoothWaveFullStrip);
	stopTask(smoothWave);
	stopTask(christmasLights);
	stopTask(fadeColors);
}

int count = 0;
int dOfWheels = 4;
int rOfRobot = 8;
float C_PI = 3.1415926;
int getEncValForDistance(int inches){//this returns the encoder value for drivetrain distance
	return (360*inches)/(dOfWheels*C_PI/2);
}
int getEncValForTurn(int degrees){//this returns how many times an encoder on the drivetrain needs to turn in relation to how far the robot needs to turn
	return (360*PI*2*rOfRobot)/(360*dOfWheels);
}
void moveForwards(int distance){//this moves the robot forwards *distance* inches

}
void moveBackwards(int distance){//this moves the robot backwards *distance* inches

}
void turnRight(int degrees){//this turns the robot to the right *degrees* degrees

}
void turnLeft(int degrees){//this turns the robot to the left *degrees* degrees

}

task auton(){//main task
	switch(count){
		case 0://first auton
			break;
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		default:
			break;
	}
}
//mgm up = 4095 mgm down 2550

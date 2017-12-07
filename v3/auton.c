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
int count = 0;
int dOfWheels = 4;
int rOfRobot = 8;
float PI = 3.1415926;
int getEncValForDistance(int inches){//this returns the encoder value for drivetrain distance
	return (360*inches)/(dOfWheels*PI/2);
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
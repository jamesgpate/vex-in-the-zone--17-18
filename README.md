
# Quantum Robotics
## \\/\\/\\/ HEY YOU! YEAH YOU! LOOK DOWN HERE! \\/\\/\\/
## My Git repo for Vex and stuff
<img src="https://github.com/notjoms/vex-in-the-zone--17-18/blob/master/page_assets/main_logo.png" height=200 width=200/>


[CSHS](https://cshs.csisd.org) Team [__8332A__](https://vexdb.io/teams/view/8332A) Quantum Robotics

This code is to run our robot.

It is MIT licensed, so you can do whatever you want so long as you credit me.

## Helpful Links and Stuff

* [Vex Forums](https://www.vexforum.com/)
* [PID help and info](https://www.vexforum.com/index.php/6465-a-pid-controller-in-robotc/0)
* [RobotC programming documentation](http://www.robotc.net/wikiarchive/VEX2)
* [Everything you need to know about motors](https://www.vexrobotics.com/motors.html)
* [A little old, but this is a pretty good overview of quad encoders](http://cdn.robotc.net/pdfs/vex/curriculum/Quadrature+Encoders.pdf)

## Tips and tricks from me to you, the programmer of your team

* For motors, positive is clockwise, negative is counterclockwise
* This is the same for encoders, so watch out
* If you are going to competition and are doing an autonomous, __BE SURE TO INCLUDE THE COMPETITION FILE!__ It is:
    ```c
    #include "Vex_Competition_Includes.c"
    ```
    and then you have to make sure that you have the 3 functions needed for the compiler to not yell at you:
    ```c
    task pre_auton(){
        //code goes here
    }
    task autonomous(){
        //code goes here
    }
    task usercontrol(){
        //code goes here
    }
    ```
    Look at my repo to see an example. I hand it off to another task in another file.

* Don't mess around with PID unless you have a lot of time on your hands and you don't mind your robot having a dance party. Maybe learn some calculus while you're at it.

* C is not like any other language that you've probably worked with. It isn't object-oriented, so don't expect to have a separate file for your controls and then include it. It doesn't mean you can create it as an object.

* Tasks in RobotC work like threads. If you `startTask(fancyAutonomous);`, and you can still operate your robot, you've done something very wrong.

* Also, `task auton()` is different from `void auton()` or `int auton()`. Tasks are special to RobotC. The latter basically inserts the code into where you called it from, while task runs it alongside. Again, basically a thread.

* Please use functions for your autonomous. Take me for example:
    ```c
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
    ```
    I can insert `moveForwards(5)` anywhere in my autonomous script and it will move forward 5 inches. Please don't copy and paste the same thing over and over. I've been there and I am never going back.

* Whenever your team wants you to program something, tell them it needs to be in the form of a deliverable. "I want to be able to do this." Have them be specific though, like "I want to have arcade drive on the drivetrain with channel 4 controlling forwards and backwards and channel 3 controlling turning." Also, have them create a sheet with the ports for the motors/sensors. This will make your life much easier.

* Those links up there are pretty helpful. You should check them out.

* __**Source control please!**__ Please use Git or some other version control system (Not Google Drive or a flash drive, stop it) and a personal thing from me, make it open source so other people can use it, like I do. [Github has some nice tutorials about Git and its usefulness.](https://guides.github.com/activities/hello-world/) [Another tutorial.](https://guides.github.com/introduction/git-handbook/)
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Blue2OpMode extends BlueOpMode {
    @Override
    protected void execute() {

        robot.drive(1,0,0,39);

        //check the rings

        //based on the number of rings do one of the following:

        this.targetA();
        this.targetB();
        this.targetC();
    }

    protected void targetA() {
        robot.drive(1,0,0,24);
        robot.drive(1,0,90,48);
        robot.drive(1,0,180,72);
        robot.drive(1,0,-100,7);
        robot.drive(1,0,-30,60);
    }

    protected void targetB() {
        robot.drive(1,0,20,48);
        robot.drive(1,0,-160,12);
        robot.drive(1,0,160,54);
        robot.drive(1,0,-90,24);
        robot.drive(1,0,-7,72);

    }

    protected void targetC() {
        robot.drive(1,0,0,78); //drive to back of target area (but leave room for wobble goal)
        //deposit first wobble goal
        robot.drive(-1,0,0,24);//back up
        robot.drive(0,1,0,48);//strafe right
        robot.drive(0,-1,-90,66);//strafe to line up with second wobble goal
        robot.drive(1,0,-90,30);//push wobble goal into position of original wobble goal
        robot.drive(-1,0,-90,6);//back up slightly
        robot.drive(-1,0,0,13);//back up to wall
        robot.drive(0,-1,0,15);//strafe left
        robot.drive(1,0,0,107);//push second wobble goal into target area
        //deposit second wobble goal
        robot.drive(-1,0,0,36);//back up to line
    }

}
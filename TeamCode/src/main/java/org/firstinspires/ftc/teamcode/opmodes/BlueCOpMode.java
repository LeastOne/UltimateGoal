package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueCOpMode extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drive(1,0,0,36); //drive up to rings
        //scan for ring amount
        robot.drive(1,0,0,78); //drive to back of target area (but leave room for wobble goal)
        //deposit first wobble goal
        robot.drive(-1,0,0,24);//back up
        robot.drive(0,1,0,54);//strafe right
        robot.drive(0,-1,90,66);//strafe to line up with second wobble goal
        robot.drive(1,0,90,36);//push wobble goal into position of original wobble goal
        robot.drive(-1,0,90,6);//back up slightly
        robot.drive(-1,0,0,13);//back up to wall
        robot.drive(0,-1,0,15);//strafe left
        robot.drive(1,0,0,107);//push second wobble goal into target area
        //deposit second wobble goal
        robot.drive(-1,0,0,36);//back up to line
    }
}
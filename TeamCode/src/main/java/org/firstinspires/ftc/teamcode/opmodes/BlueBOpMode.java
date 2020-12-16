package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueBOpMode extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drive(1,0,0,24);
        robot.drive(1,0,-10,60);
        robot.drive(-1,0,-10,60);
        robot.drive(-1,0,0,24);
        robot.drive(0,1,0,33);
        robot.drive(1,0,0,42);
        robot.drive(1,0,7,32);
    }
}
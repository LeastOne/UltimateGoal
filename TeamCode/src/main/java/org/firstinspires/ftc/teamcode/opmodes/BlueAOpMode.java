package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueAOpMode extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drive(1,0,0,34);
        robot.drive(1,0,0,24);
        robot.drive(1,0,90,48);
        robot.drive(1,0,180,72);
        robot.drive(1,0,-100,7);
        robot.drive(1,0,-30,60);
    }
}
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueAOpMode extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drive(1,0,1 ,34);
        robot.drive(1,0,0,24);
        robot.drive(-1,0,0,58);
        robot.drive(0,1,0,36);
        robot.drive(1,0,25,69);
    }
}
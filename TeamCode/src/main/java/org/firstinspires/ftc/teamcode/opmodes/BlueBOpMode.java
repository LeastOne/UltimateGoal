package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueBOpMode extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drive(1,0,0,24);
        robot.drive(1,0,-20,48);
        robot.drive(1,0,160,12);
        robot.drive(1,0,-160,54);
        robot.drive(1,0,90,24);
        robot.drive(1,0,7,72);

    }
}
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Blue2OpMode extends BlueOpMode {
    @Override
    protected void execute() {
        this.targetA();
        this.targetB();
    }

    protected void targetA() {
        robot.drive(1,0,0,34);
        robot.drive(1,0,0,24);
        robot.drive(1,0,90,48);
        robot.drive(1,0,180,72);
        robot.drive(1,0,-100,7);
        robot.drive(1,0,-30,60);
    }

    protected void targetB() {
        robot.drive(1,0,0,24);
        robot.drive(1,0,20,48);
        robot.drive(1,0,-160,12);
        robot.drive(1,0,160,54);
        robot.drive(1,0,-90,24);
        robot.drive(1,0,-7,72);

    }

}
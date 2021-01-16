package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Blue2OpMode extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drivePower = 0.6;

        robot.drive(1,0,0,37);

        String recognitionLabel = "";
        for (int i = 0; i < 60; i++) {
            sleep(50);
            if (robot.recognitions != null && !robot.recognitions.isEmpty()) {
                recognitionLabel = robot.recognitions.get(0).getLabel();
            }
        }

        switch (recognitionLabel) {
            case "Quad": this.targetC(); break;
            case "Single": this.targetB(); break;
            default: this.targetA(); break;
        }
    }

    protected void targetA() {
        robot.drive(1,0,0,26);
        robot.drive(-1,0,0,64);
        robot.drive(0,1,0,36);
        robot.drive(1,0,30,64);
        robot.drive(-1,0,30,5);
        robot.drive(0,1,30,24);
    }

    protected void targetB() {
        robot.drive(1,0,-15,51);
        robot.drive(-1,0,-20,44);
        robot.drive(-1,0,0,42);
        robot.drive(0,1,0,35);
        robot.drive(1,0,0,42);
        robot.drive(1,0,9,36);
        robot.drive(-1,0,9,6);
    }

    protected void targetC() {
        robot.drive(1,0,0,73);
        robot.drive(-1,0,0,111);
        robot.drive(0,1,0,34);
        robot.drive(1,0,0,42);
        robot.drive(1,0,22,67);
        robot.drive(-1,0,22,36);
    }
}
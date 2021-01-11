package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Blue2OpMode extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drive(1,0,0,38);

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
        robot.drive(1,0,0,27);
        robot.drive(-1,0,0,66);
        robot.drive(0,1,0,36);
        robot.drive(1,0,30,64);
        robot.drive(-1,0,30,5);
        robot.drive(0,1,30,24);
    }

    protected void targetB() {
        robot.drive(1,0,-20,48);
        robot.drive(-1,0,-20,44);
        robot.drive(-1,0,0,40);
        robot.drive(0,1,0,32);
        robot.drive(1,0,0,42);
        robot.drive(1,0,9,36);
    }

    protected void targetC() {
        robot.drivePower = 0.6;
        robot.drive(1,0,0,76);
        robot.drive(-1,0,0,114);
        robot.drive(0,1,0,34);
        robot.drive(1,0,0,42);
        robot.drive(1,0,22,67);
        robot.drive(-1,0,22,36);
    }
}
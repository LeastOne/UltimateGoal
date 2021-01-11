package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Blue2OpMode extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drive(1,0,0,36);

        String recognitionLabel = "";
        for (int i = 0; i < 60; i++){
            sleep(50);
            if (robot.recognitions != null && !robot.recognitions.isEmpty()) {
                recognitionLabel = robot.recognitions.get(0).getLabel();
            }
        }

        switch (recognitionLabel){
            case "Quad": this.targetC(); break;
            case "Single": this.targetB(); break;
            default: this.targetA(); break;
        }
    }

    protected void targetA() {
        robot.drive(1,0,0,29);
        robot.drive(-1,0,0,65);
        robot.drive(0,1,0,36);
        robot.drive(1,0,25,69);
    }

    protected void targetB() {
        robot.drive(1,0,-20,48);
        robot.drive(-1,0,-20,44);
        robot.drive(-1,0,0,40);
        robot.drive(0,1,0,33);
        robot.drive(1,0,0,42);
        robot.drive(1,0,7,36);
    }

    protected void targetC() {
        robot.drive(1,0,0,78);
        robot.drive(-1,0,0,114);
        robot.drive(0,1,0,32);
        robot.drive(1,0,0,42);
        robot.drive(1,0,17,70);

    }
}
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
        robot.drive(1,0,0,22);
        robot.drive(-1,0,0,58);
        robot.drive(0,1,0,36);
        robot.drive(1,0,25,69);
    }

    protected void targetB() {
        robot.drive(1,0,-15,48);
        robot.drive(-1,0,-15,44);
        robot.drive(-1,0,0,40);
        robot.drive(0,1,0,33);
        robot.drive(1,0,0,42);
        robot.drive(1,0,7,32);
    }

    protected void targetC() {
        robot.drive(1,0,0,78); // drive to back of target area (but leave room for wobble goal)
        robot.drive(-1,0,0,24); // back up
        robot.drive(0,1,0,54); // strafe right
        robot.drive(0,-1,90,66); // strafe to line up with second wobble goal
        robot.drive(1,0,90,36); // push wobble goal into position of original wobble goal
        robot.drive(-1,0,90,6); // back up slightly
        robot.drive(-1,0,0,13); // back up to wall
        robot.drive(0,-1,0,15); // strafe left
        robot.drive(1,0,0,107); // push second wobble goal into target area
        robot.drive(-1,0,0,36); // back up to line
    }
}
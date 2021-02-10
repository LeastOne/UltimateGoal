package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.internal.Robot.ShooterMode.ON;
import static org.firstinspires.ftc.teamcode.internal.Robot.ShooterMode.SHOOT;

@Autonomous
public class Blue2OpMode extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drivePower = 1.0;

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

        //shootPowerShots();
    }

    private void shootPowerShots() {
        robot.shooter(ON);
        robot.drive(1,0,-10,0); //robot turns to (guessed) orientation to shoot power shot target
        robot.shooter(SHOOT);
        robot.drive(1,0,-15,0);
        robot.shooter(SHOOT);
        robot.drive(1,0,-20,0);
        robot.shooter(SHOOT);
        robot.drive(1,0,0,8); //make sure to get back on line
    }

    protected void targetA() {
        robot.drive(1,0,0,26);
        robot.drive(-1,0,0,64);
        robot.drive(0,1,0,36);
        robot.drive(1,0,30,64);
        robot.drive(-1,0,30,5);
        robot.drive(0,1,30,24);
        robot.turn(1, 0);
        robot.drive(0,1,0,4);
    }

    protected void targetB() {
        robot.drive(1,0,-15,51);
        robot.drive(-1,0,-15,51);
        robot.drive(-1,0,0,34);
        robot.drive(0,1,0,35);
        robot.drive(1,0,0,42);
        robot.drive(1,0,9,40);
        robot.drive(-1,0,9,17);
        robot.drive(0,1,0,1);

    }

    protected void targetC() {
        robot.drive(-1,0,22,44); //robot backs up a little bit more to move it behind the line
        robot.drive(1,0,0,73);
        robot.drive(-1,0,0,111);
        robot.drive(0,1,0,34);
        robot.drive(1,0,0,42);
        robot.drive(1,0,25,67);
    }
}
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
        for (int i = 0; i < 50; i++) {
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

        this.shootPowerShots();
    }

    protected void targetA() {
        robot.drive(1,0,0,26);
        robot.drive(-1,0,0,60);
        robot.drive(0,1,0,36);
        robot.drive(1,0,28,64);
        robot.drive(-1,0,28,6.5); //if this goes wrong then change it back to 5 and change line #40
        robot.drive(0,1,30,28);
        robot.drive(-1,1,0,2);
    }

    protected void targetB() {
        robot.drive(1,0,-15,51);
        robot.drive(-1,0,-15,51);
        robot.drive(-1,0,0,30);
        robot.drive(0,1,0,35);
        robot.drive(1,0,0,42);
        robot.drive(1,0,9,40);
        robot.drive(-1,0,9,17);
        robot.drive(0,1,0,1);
    }

    protected void targetC() {
        robot.drive(1,0,0,73);
        robot.drive(-1,0,0,110);
        robot.drive(0,1,0,34);
        robot.drive(1,0,0,42);
        robot.drive(1,0,25,67);
        robot.drive(-1,0,22,42); //robot backs up a little bit more to move it behind the line
        robot.drive(0,1,0,10.5); //changed from 12 to 10.5
    }

    private void shootPowerShots() {
        robot.shooter(ON);
        robot.drive(1,0,-2,0); //robot turns to (guessed) orientation to shoot power shot target
        robot.shooter(SHOOT);
        robot.drive(1,0,-7,0);
        robot.shooter(SHOOT);
        robot.drive(1,0,-12,0);
        robot.shooter(SHOOT);
        robot.drive(1,0,0,12); //make sure to get back on line
    }
}
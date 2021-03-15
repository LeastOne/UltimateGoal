package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeLiftMode.DOWN;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeLiftMode.UP;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeMode.OFF;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeMode.ON;

public class IntakeController extends RobotController {
    public IntakeController(OpMode opMode){
        super(opMode);
    }


    public void execute(){
        if (gamepad2.right_trigger > .5) robot.intake(ON);
        else if (gamepad2.left_trigger > .5) robot.intake(OFF);
        else if (gamepad2.right_stick_button) robot.intake(UP);
        else if (gamepad2.left_stick_button) robot.intake(DOWN);
    }
}

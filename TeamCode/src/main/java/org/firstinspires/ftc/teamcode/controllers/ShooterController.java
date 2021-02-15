package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.internal.Robot;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmAction.DOWN;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmAction.STOP;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmAction.UP;

public class ShooterController extends RobotController {
    public ShooterController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        robot.shooter(-gamepad2.left_stick_y,gamepad2.right_stick_x);
    }
}
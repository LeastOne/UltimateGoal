package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.ClawPosition.CLOSE;
import static org.firstinspires.ftc.teamcode.internal.Robot.ClawPosition.OPEN;

public class ClawController extends RobotController {
    public ClawController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.left_bumper) robot.claw(OPEN);
        if (gamepad2.right_bumper) robot.claw(CLOSE);
    }
}

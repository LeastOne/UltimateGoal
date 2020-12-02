package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class DriveController extends RobotController {
    public DriveController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        robot.drive(
           -gamepad1.left_stick_y,
            gamepad1.left_stick_x,
            gamepad1.right_stick_x
        );
    }
}
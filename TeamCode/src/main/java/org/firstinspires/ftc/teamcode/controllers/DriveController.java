package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class DriveController extends RobotController {
    public DriveController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad1.dpad_up) robot.drivePower = 1;
        else if (gamepad1.dpad_down) robot.drivePower = .5;
        robot.drive(
           -gamepad1.left_stick_y,
            gamepad1.left_stick_x,
            gamepad1.right_stick_x
        );
    }
}
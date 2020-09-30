package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class DriveController extends RobotController {
    private static final double POWER = 0.67;

    public DriveController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        robot.drive(drive * POWER, turn * POWER);
    }
}

package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class DriveController extends RobotController {
    private static final double POWER = 0.67;

    public DriveController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if(robot.mecanumMode) {
            driveMecanum();
        } else{
            driveStandard();
        }
    }

    private void driveStandard() {
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double left = POWER * (drive+turn);
        double right = POWER * (drive-turn);

        double max = Math.max(Math.abs(left), Math.abs(right));

        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        robot.drive(left, left, right, right);
    }

    private void driveMecanum() {
        // since left stick can be pushed in all directions to control the robot's movements, its "power" must be the actual
        // distance from the center, or the hypotenuse of the right triangle formed by left_stick_x and left_stick_y
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

        // angle between x axis and "coordinates" of left stick
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;

        // turn
        double rightX = gamepad1.right_stick_x;

        double lf = POWER * (r * Math.cos(robotAngle) + rightX);
        double lr = POWER * (r * Math.sin(robotAngle) + rightX);
        double rf = POWER * (r * Math.sin(robotAngle) - rightX);
        double rr = POWER * (r * Math.cos(robotAngle) - rightX);

        robot.drive(lf, lr, rf, rr);
    }
}
package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.BACK;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.TILTED;
import static org.firstinspires.ftc.teamcode.internal.Robot.TiltPosition.UP;

public class TiltController extends RobotController {
    public TiltController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.left_trigger > 0) robot.tilt(gamepad2.left_trigger);
        else if (gamepad2.right_trigger > 0) robot.tilt(-gamepad2.right_trigger);
        else if (gamepad2.y) robot.tiltAsync(UP);
        else if (gamepad2.b) robot.tiltAsync(TILTED);
        else if (gamepad2.a) robot.tiltAsync(BACK);
        else robot.tilt(0);
    }
}

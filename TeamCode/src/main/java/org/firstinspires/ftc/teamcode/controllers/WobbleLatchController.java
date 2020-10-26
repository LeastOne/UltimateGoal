package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.internal.Robot;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class WobbleLatchController extends RobotController {
    public WobbleLatchController(OpMode opMode) {
        super(opMode);
        }

    @Override
    public void execute() {
        if (gamepad2.dpad_left) robot.wobbleLatch(Robot.WobbleLatchPosition.CLOSED);
        if (gamepad2.dpad_right) robot.wobbleLatch(Robot.WobbleLatchPosition.OPEN);
    }

}

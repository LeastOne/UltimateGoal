package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmPosition.DOWN;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmPosition.UP;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleLatchPosition.CLOSED;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleLatchPosition.OPEN;

public class WobbleController extends RobotController {
    public WobbleController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.left_bumper) robot.wobbleArm(DOWN);
        if (gamepad2.right_bumper) robot.wobbleArm(UP);
        if (gamepad2.dpad_left) robot.wobbleLatch(CLOSED);
        if (gamepad2.dpad_right) robot.wobbleLatch(OPEN);
    }
}
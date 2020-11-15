package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmPosition.DOWN;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmPosition.STOP;
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
        else if (gamepad2.right_bumper) robot.wobbleArm(UP);
        else robot.wobbleArm(STOP);
        if (gamepad2.dpad_left) robot.wobbleLatch(CLOSED);
        else if (gamepad2.dpad_right) robot.wobbleLatch(OPEN);
    }
}
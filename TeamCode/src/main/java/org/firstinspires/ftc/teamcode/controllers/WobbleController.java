package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.internal.Robot;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmAction.DOWN;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmAction.STOP;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmAction.UP;

public class WobbleController extends RobotController {
    public WobbleController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.left_bumper) robot.wobbleArm(DOWN);
        else if (gamepad2.right_bumper) robot.wobbleArm(UP);
        else robot.wobbleArm(STOP);
        if (gamepad2.dpad_left) robot.wobbleLatch(Robot.WobbleLatchPosition.CLOSED);
        else if (gamepad2.dpad_right) robot.wobbleLatch(Robot.WobbleLatchPosition.OPEN);
        else if (gamepad2.dpad_up) robot.wobbleRingLatch(Robot.WobbleRingLatchPosition.CLOSED);
        else if (gamepad2.dpad_down) robot.wobbleRingLatch(Robot.WobbleRingLatchPosition.OPEN);
    }
}
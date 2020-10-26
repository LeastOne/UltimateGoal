package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class WobbleArmController extends RobotController{

    public WobbleArmController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
     if (gamepad2.left_bumper)robot.wobbleArm();
     if (gamepad2.right_bumper)robot.wobbleArm();


    }
}

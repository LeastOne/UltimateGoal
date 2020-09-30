package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class LiftController extends RobotController {
    private double power = 0.5;

    public LiftController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.dpad_up) robot.lift(power);
        else if (gamepad2.dpad_down) robot.lift(-power);
        else robot.lift( 0);
    }
}

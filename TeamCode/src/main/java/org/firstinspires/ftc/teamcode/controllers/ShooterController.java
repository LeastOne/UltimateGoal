package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.ShooterMode.OFF;
import static org.firstinspires.ftc.teamcode.internal.Robot.ShooterMode.ON;
import static org.firstinspires.ftc.teamcode.internal.Robot.ShooterMode.SHOOT;

public class ShooterController extends RobotController {
    public ShooterController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.x) robot.shooter(ON);
        else if (gamepad2.y) robot.shooter(SHOOT);
        else if (gamepad2.b) robot.shooter(OFF);

    }
}
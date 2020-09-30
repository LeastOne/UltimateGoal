package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.SlidePosition.IN;
import static org.firstinspires.ftc.teamcode.internal.Robot.SlidePosition.OUT;

public class SlideController extends RobotController {
    public SlideController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute(){
        if (gamepad2.dpad_left) robot.slide(OUT);
        else if (gamepad2.dpad_right) robot.slide(IN);
    }
}

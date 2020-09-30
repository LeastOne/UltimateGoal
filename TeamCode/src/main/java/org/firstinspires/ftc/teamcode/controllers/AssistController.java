package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class AssistController extends RobotController {
    public AssistController(OpMode opMode) {
        super(opMode);
    }
    @Override
    public void execute() {
        if(gamepad2.x) robot.pickUpStone(false);
    }
}

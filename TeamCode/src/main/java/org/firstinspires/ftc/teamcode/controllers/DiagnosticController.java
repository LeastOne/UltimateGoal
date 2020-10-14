package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class DiagnosticController extends RobotController {
    public DiagnosticController(OpMode opMode) { super(opMode); }

    @Override
    public void execute() {
        if ((gamepad1.back && gamepad1.start && !gamepad1.atRest()) ||
            (gamepad2.back && gamepad2.start && !gamepad2.atRest())) {
            robot.diagnosticMode = true;
        }

        if (robot.diagnosticMode){
            robot.setAttachmentMotorPower(
                gamepad2.left_stick_x,
                gamepad2.left_stick_y,
                gamepad2.right_stick_x,
                gamepad2.right_stick_y
            );
        }
    }
}
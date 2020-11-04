package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Blue1OpMode extends BlueOpMode {
    @Override
    protected void execute() {
        gamepad1.start = true;
        gamepad1.back = true;
        gamepad1.right_trigger = 1;
        gamepad1.x = true;
        super.execute();
    }
}
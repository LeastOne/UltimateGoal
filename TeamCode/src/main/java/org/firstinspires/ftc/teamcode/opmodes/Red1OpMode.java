package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Red1OpMode extends RedOpMode {
    @Override
    protected void execute() {
        gamepad1.back = true;
        gamepad1.start = true;
        gamepad1.left_trigger = 1;
        gamepad1.y = true;
        super.execute();
    }
}

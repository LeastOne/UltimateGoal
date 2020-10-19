package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Red1OpMode extends RedOpMode {
    @Override
    protected void execute() {
        gamepad1.a = true;
        super.execute();
    }
}

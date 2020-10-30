package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Blue1OpMode extends BlueOpMode {
    @Override
    protected void execute() {
        gamepad1.a = true;
        super.execute();
    }
}
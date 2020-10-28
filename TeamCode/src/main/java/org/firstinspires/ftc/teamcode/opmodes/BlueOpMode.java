package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.internal.Alliance;

import static org.firstinspires.ftc.teamcode.internal.Alliance.BLUE;

public abstract class BlueOpMode extends TeleOpMode {
    @Override
    protected Alliance getAlliance() {
        return BLUE;
    }
}
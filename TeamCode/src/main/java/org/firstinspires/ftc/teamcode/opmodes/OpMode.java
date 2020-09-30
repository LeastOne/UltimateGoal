package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.controllers.RobotController;
import org.firstinspires.ftc.teamcode.internal.Alliance;
import org.firstinspires.ftc.teamcode.internal.Robot;

import static org.firstinspires.ftc.teamcode.internal.Alliance.UNKNOWN;

public abstract class OpMode extends LinearOpMode {
    private boolean calibrate = false;

    public Robot robot;

    private RobotController[] robotControllers;

    public OpMode() {
        this(true);
    }

    public OpMode(boolean calibrate) {
        this.calibrate = calibrate;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.init(getAlliance());
        if (calibrate) robot.calibrate();
        waitForStart();
        sleep(250);
        robot.start();
        sleep(250);
        execute();
    }

    public boolean isActive() {
        yield();
        return opModeIsActive();
    }

    public boolean isContinuing() {
        yield();
        return !isStopRequested() && !gamepad1.back && !gamepad2.back;
    }

    protected Alliance getAlliance() {
        return UNKNOWN;
    }

    protected abstract void execute();

    private void yield() {
        Thread.yield();
    }
}

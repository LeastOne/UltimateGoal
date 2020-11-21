package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.controllers.RobotController;
import org.firstinspires.ftc.teamcode.internal.Alliance;
import org.firstinspires.ftc.teamcode.internal.Robot;

import static org.firstinspires.ftc.teamcode.internal.Alliance.UNKNOWN;

public abstract class OpMode extends LinearOpMode {
    private boolean calibrate = false;

    private RobotController[] robotControllers;

    public Gamepad gamepad1 = new Gamepad();

    public Gamepad gamepad2 = new Gamepad();

    public Robot robot;

    public OpMode() {
        this(true);
    }

    public OpMode(boolean calibrate) {
        this.calibrate = calibrate;
    }

    @Override
    public void runOpMode() {
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
        Thread.yield();

        // Creating our own copy of the gamepads to ensure they
        // don't change in between controller invocations.
        try {
            this.gamepad1.copy(super.gamepad1);
            this.gamepad2.copy(super.gamepad2);
        } catch (Exception e) {
            e.printStackTrace();
        }

        return opModeIsActive();
    }

    public boolean isStopping() {
        Thread.yield();
        return isStopRequested() || gamepad1.back || gamepad2.back;
    }

    protected Alliance getAlliance() {
        return UNKNOWN;
    }

    protected abstract void execute();
}
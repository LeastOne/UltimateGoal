package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import java.util.List;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLACK;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLUE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.GRAY;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.GREEN;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.YELLOW;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.teamcode.internal.Robot.RobotDriveType.MECANUM;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmAction.DOWN;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmAction.UP;

public class Robot {
    public double drivePower = 0.5;
    private static final double INCHES_PER_ROTATION = 3.95 * Math.PI;
    private static final double TICKS_PER_INCH = 537.6 / INCHES_PER_ROTATION;

    private static RevBlinkinLedDriver.BlinkinPattern DEFAULT_COLOR = GRAY;
    private static RevBlinkinLedDriver.BlinkinPattern CALIBRATE_COLOR = RAINBOW_LAVA_PALETTE;
    private static RevBlinkinLedDriver.BlinkinPattern READY_COLOR = HEARTBEAT_GRAY;
    private static RevBlinkinLedDriver.BlinkinPattern SEARCHING_COLOR = LIGHT_CHASE_GRAY;
    private static RevBlinkinLedDriver.BlinkinPattern PICKUP_COLOR = GREEN;
    private static RevBlinkinLedDriver.BlinkinPattern TARGET_COLOR = YELLOW;

    public boolean diagnosticMode;

    private OpMode opMode;

    private BNO055IMU imu;

    public enum RobotDriveType {
        STANDARD, MECANUM
    }

    private RobotDriveType driveType = MECANUM;

    private DcMotor driveLeftFront;
    private DcMotor driveRightFront;
    private DcMotor driveLeftRear;
    private DcMotor driveRightRear;

    private DcMotor intakeTop;
    private DcMotor intakeBottom;
    private DcMotor wobbleArm;

    private Servo wobbleLatch;
    private Servo wobbleRingLatch;
    private RevBlinkinLedDriver lights;

    private DigitalChannel wobbleLimit;
    private DigitalChannel wobbleTouch;

    private VisionThread visionThread;

    public WebcamName webcamName;
    public int cameraMonitorViewId;
    public int tfodMonitorViewId;

    public boolean navigationTargetVisible = false;
    public Position position = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    public Orientation orientation = new Orientation();

    public boolean itemVisible = false;
    public Position itemPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    public Orientation itemOrientation = new Orientation();

    public List<Recognition> recognitions = null;

    public String error;

    public boolean mecanumMode = true;

    public Robot(OpMode opMode) {
        this.opMode = opMode;
    }

    public void init(Alliance alliance) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        drivePower = 0.5;

        driveLeftFront = hardwareMap.get(DcMotor.class, "driveLeftFront");
        driveLeftFront.setDirection(REVERSE);
        driveLeftFront.setZeroPowerBehavior(BRAKE);
        driveLeftFront.setMode(STOP_AND_RESET_ENCODER);
        driveLeftFront.setMode(RUN_USING_ENCODER);

        driveRightFront = hardwareMap.get(DcMotor.class,"driveRightFront");
        driveRightFront.setDirection(FORWARD);
        driveRightFront.setZeroPowerBehavior(BRAKE);
        driveRightFront.setMode(STOP_AND_RESET_ENCODER);
        driveRightFront.setMode(RUN_USING_ENCODER);

        driveLeftRear = hardwareMap.get(DcMotor.class,"driveLeftRear");
        driveLeftRear.setDirection(REVERSE);
        driveLeftRear.setZeroPowerBehavior(BRAKE);
        driveLeftRear.setMode(STOP_AND_RESET_ENCODER);
        driveLeftRear.setMode(RUN_USING_ENCODER);

        driveRightRear = hardwareMap.get(DcMotor.class, "driveRightRear");
        driveRightRear.setDirection(FORWARD);
        driveRightRear.setZeroPowerBehavior(BRAKE);
        driveRightRear.setMode(STOP_AND_RESET_ENCODER);
        driveRightRear.setMode(RUN_USING_ENCODER);

        intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");
        intakeTop.setDirection(FORWARD);
        intakeTop.setZeroPowerBehavior(BRAKE);
        intakeTop.setMode(STOP_AND_RESET_ENCODER);
        intakeTop.setMode(RUN_USING_ENCODER);

        intakeBottom = hardwareMap.get(DcMotor.class, "intakeBottom");
        intakeBottom.setDirection(FORWARD);
        intakeBottom.setZeroPowerBehavior(BRAKE);
        intakeBottom.setMode(STOP_AND_RESET_ENCODER);
        intakeBottom.setMode(RUN_USING_ENCODER);

        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");
        wobbleArm.setDirection(FORWARD);
        wobbleArm.setZeroPowerBehavior(BRAKE);
        wobbleArm.setMode(STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(RUN_USING_ENCODER);

        wobbleLatch = hardwareMap.get(Servo.class,"wobbleLatch");
        wobbleRingLatch = hardwareMap.get(Servo.class,"wobbleRingLatch");
        lights = hardwareMap.get(RevBlinkinLedDriver.class,"lights");

        wobbleLimit = hardwareMap.get(DigitalChannel.class, "wobbleLimit");
        wobbleLimit.setMode(INPUT);
        wobbleTouch = hardwareMap.get(DigitalChannel.class, "wobbleTouch");
        wobbleTouch.setMode(INPUT);

        try {
            webcamName = hardwareMap.get(WebcamName.class,"Webcam 1");
            cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId","id",hardwareMap.appContext.getPackageName());

            visionThread = new VisionThread(opMode,this);
            visionThread.start();
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (alliance == Alliance.RED) {
            DEFAULT_COLOR = RED;
            READY_COLOR = HEARTBEAT_RED;
            SEARCHING_COLOR = LIGHT_CHASE_RED;
        }

        if (alliance == Alliance.BLUE) {
            DEFAULT_COLOR = BLUE;
            READY_COLOR = HEARTBEAT_BLUE;
            SEARCHING_COLOR = LIGHT_CHASE_BLUE;
        }
    }

    public void calibrate() {
        setLights(CALIBRATE_COLOR);
        setLights(READY_COLOR);
    }

    public void start() {
        setLights(DEFAULT_COLOR);
    }

    public void drive(double drive, double strafe, double turn) {
        if (opMode.isStopping()) return;

        if (driveType != MECANUM) strafe = 0;

        // since left stick can be pushed in all directions to control the robot's movements, its "power" must be the actual
        // distance from the center, or the hypotenuse of the right triangle formed by left_stick_x and left_stick_y
        double r = Math.hypot(strafe, drive);

        // angle between x axis and "coordinates" of left stick
        double robotAngle = Math.atan2(drive, strafe) - Math.PI / 4;

        double lf = drivePower * (r * Math.cos(robotAngle) + turn);
        double lr = drivePower * (r * Math.sin(robotAngle) + turn);
        double rf = drivePower * (r * Math.sin(robotAngle) - turn);
        double rr = drivePower * (r * Math.cos(robotAngle) - turn);

        driveLeftFront.setPower(lf);
        driveRightFront.setPower(rf);
        driveLeftRear.setPower(lr);
        driveRightRear.setPower(rr);
    }

    public void drive(double drive, double strafe, double heading, double inches) {
        if (opMode.isStopping()) return;

        turn(drive + strafe, heading);

        resetEncoders();

        int targetPosition = (int)(inches * TICKS_PER_INCH);
        int position = 0;

        double remainder, turn;

        while (!opMode.isStopping() && targetPosition - position > 0) {
            remainder = getRemainderLeftToTurn(heading);
            if (drive != 0) drive = clamp(0.2, drive, (1 - (double)position / targetPosition) * TICKS_PER_INCH * 12);
            if (strafe != 0) strafe = clamp(0.2, strafe, (1 - (double)position / targetPosition) * TICKS_PER_INCH * 12);
            turn = remainder / 45;
            drive(drive, strafe, turn);

            position = (
                Math.abs(driveLeftFront.getCurrentPosition()) +
                Math.abs(driveLeftRear.getCurrentPosition()) +
                Math.abs(driveRightFront.getCurrentPosition()) +
                Math.abs(driveRightRear.getCurrentPosition())
            ) / 4;
        }

        this.drive(0, 0, 0);
    }

    public void turn(double power, double heading) {
        if (opMode.isStopping()) return;

        power = Math.abs(power);

        double remainder, turn;

        do {
            remainder = getRemainderLeftToTurn(heading);
            turn = clamp(0.2, power, remainder / 45 * power);
            drive(0, 0,turn);
        } while (!opMode.isStopping() && (remainder < -1 || remainder > 1));

        drive(0, 0, 0);
    }

    public void setLights(RevBlinkinLedDriver.BlinkinPattern pattern) {
        lights.setPattern(pattern == BLACK ? DEFAULT_COLOR : pattern);
    }

    private double getOffset(Recognition item) {
        // Linear Coordinates
        final double x1 = 140/*height*/, y1 = 14/*degrees*/;
        final double x2 = 254/*height*/, y2 = 9/*degrees*/;

        // Linear Equation: y(x) = y1 + ((y2 - y1) / (x2 - x1)) * (x - x1)
        return y1 + ((y2 - y1) / (x2 - x1)) * (item.getHeight() - x1);
    }

    public void setAttachmentMotorPower(double power0, double power1, double power2, double power3) {
        intakeTop.setPower(power0);
        intakeBottom.setPower(power1);
        wobbleArm.setPower(power2);
    }

    public enum WobbleArmAction {
        UP(0.50), DOWN(-0.50), STOP(0);

        public double power;

        WobbleArmAction(double power) {
            this.power = power;
        }
    }

    public void wobbleArm(WobbleArmAction action) {
        if ((action == UP && !wobbleLimit.getState()) ||
            (action == DOWN && !wobbleTouch.getState())) {
            wobbleArm.setPower(0);
        } else {
            wobbleArm.setPower(action.power);
        }
    }

    public enum WobbleArmPosition {
        DOWN(0), UP(1275), BACK(2550);

        public int value;

        WobbleArmPosition(int value) {
            this.value = value;
        }
    }

    public void wobbleArm(WobbleArmPosition position) {
        wobbleArm.setPower(0.50);
        wobbleArm.setTargetPosition(position.value);
        wobbleArm.setMode(RUN_TO_POSITION);
        while (!opMode.isStopping() && wobbleArm.isBusy()) opMode.sleep(50);
        wobbleArm.setPower(0);
        wobbleArm.setMode(RUN_USING_ENCODER);
    }

    public enum WobbleLatchPosition {
        OPEN(0.3), CLOSED(0.22);

        public double value;

        WobbleLatchPosition(double value) {
            this.value = value;
        }
    }

    public void wobbleLatch(WobbleLatchPosition position) {
        wobbleLatch.setPosition(position.value);
    }

    public enum WobbleRingLatchPosition {
        OPEN(1), CLOSED(0);

        public double value;

        WobbleRingLatchPosition(double value) {
            this.value = value;
        }
    }

    public void wobbleRingLatch(WobbleRingLatchPosition position) {
        wobbleRingLatch.setPosition(position.value);
    }
    public void addTelemetry() {
        Telemetry telemetry = opMode.telemetry;

        orientation = getOrientation();

        telemetry.addData("Drive", "%.2f Pow", opMode.gamepad2.left_stick_y);
        telemetry.addData("Turn", "%.2f Pow", opMode.gamepad2.right_stick_x);
        telemetry.addData("Drive (LF)", "%.2f Pow, %d Pos", driveLeftFront.getPower(), driveLeftFront.getCurrentPosition());
        telemetry.addData("Drive (LR)", "%.2f Pow, %d Pos", driveLeftRear.getPower(), driveLeftRear.getCurrentPosition());
        telemetry.addData("Drive (RF)", "%.2f Pow, %d Pos", driveRightFront.getPower(), driveRightFront.getCurrentPosition());
        telemetry.addData("Drive (RR)", "%.2f Pow, %d Pos", driveRightRear.getPower(), driveRightRear.getCurrentPosition());
        telemetry.addData("Wobble Arm", "%.2f Pow, %d Pos", wobbleArm.getPower(), wobbleArm.getCurrentPosition());
        telemetry.addData("Wobble Arm Down Limit", wobbleLimit.getState());
        telemetry.addData("Wobble Arm Up Limit", wobbleTouch.getState());
        telemetry.addData("Wobble Latch Position", wobbleLatch.getPosition());
        telemetry.addData("Target Visible", navigationTargetVisible);
        telemetry.addData("Position (in)", position);
        telemetry.addData("Orientation", orientation);
        telemetry.addData("Item Visible", itemVisible);
        telemetry.addData("Item Position (in)", itemPosition);
        telemetry.addData("Item Orientation", itemOrientation);

        telemetry.addLine();

        if (recognitions != null) {
            telemetry.addData("Recognitions", recognitions.size());

            for (Recognition recognition : recognitions) {
                telemetry.addData(" Label", recognition.getLabel());
                telemetry.addData("  Left,Top", "%.3f , %.3f", recognition.getLeft(), recognition.getTop());
                telemetry.addData("  Right,Bottom", "%.3f , %.3f", recognition.getRight(), recognition.getBottom());
                telemetry.addData("  Height,Width", "%.3f , %.3f", recognition.getHeight(), recognition.getWidth());
                telemetry.addData("  Angle", "%.3f", recognition.estimateAngleToObject(DEGREES));
                telemetry.addData("  Offset", "%.3f", getOffset(recognition));
                telemetry.addData("  Heading", "%.3f", recognition.estimateAngleToObject(DEGREES) + getOffset(recognition));
                telemetry.addData("  Area", "%.3f", recognition.getWidth() * recognition.getHeight());
            }
        }

        if (error != null && !error.isEmpty())
            telemetry.addData("Error", error);
    }

    public Orientation getOrientation() {
        return imu.getAngularOrientation(INTRINSIC, ZYX, DEGREES);
    }

    private double getRemainderLeftToTurn(double heading) {
        double remainder;
        orientation = getOrientation();
        remainder = orientation.firstAngle - heading;
        if (remainder > +180) remainder -= 360;
        if (remainder < -180) remainder += 360;
        return remainder;
    }

    private void resetEncoders() {
        driveLeftFront.setMode(STOP_AND_RESET_ENCODER);
        driveLeftFront.setMode(RUN_USING_ENCODER);
        driveLeftRear.setMode(STOP_AND_RESET_ENCODER);
        driveLeftRear.setMode(RUN_USING_ENCODER);
        driveRightFront.setMode(STOP_AND_RESET_ENCODER);
        driveRightFront.setMode(RUN_USING_ENCODER);
        driveRightRear.setMode(STOP_AND_RESET_ENCODER);
        driveRightRear.setMode(RUN_USING_ENCODER);
    }

    private double clamp(double min, double max, double value) {
        return value >= 0 ?
            Math.min(max, Math.max(min, value)) :
            Math.min(-min, Math.max(-max, value));
    }
}
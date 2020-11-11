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
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

public class Robot {
    private static final double INCHES_PER_ROTATION = 3.95 * Math.PI;
    private static final double TICKS_PER_INCH = 1120 / INCHES_PER_ROTATION;

    private static RevBlinkinLedDriver.BlinkinPattern DEFAULT_COLOR = GRAY;
    private static RevBlinkinLedDriver.BlinkinPattern CALIBRATE_COLOR = RAINBOW_LAVA_PALETTE;
    private static RevBlinkinLedDriver.BlinkinPattern READY_COLOR = HEARTBEAT_GRAY;
    private static RevBlinkinLedDriver.BlinkinPattern SEARCHING_COLOR = LIGHT_CHASE_GRAY;
    private static RevBlinkinLedDriver.BlinkinPattern PICKUP_COLOR = GREEN;
    private static RevBlinkinLedDriver.BlinkinPattern TARGET_COLOR = YELLOW;

    public boolean diagnosticMode;

    private OpMode opMode;

    private BNO055IMU imu;

    private DcMotor ch_drive_lf;
    private DcMotor ch_drive_lr;
    private DcMotor ch_drive_rf;
    private DcMotor ch_drive_rr;

    private DcMotor eh_motor_0;
    private DcMotor eh_motor_1;
    private DcMotor eh_motor_2;
    private DcMotor eh_motor_3;

    private Servo eh_servo_0;
    private RevBlinkinLedDriver eh_servo_5;

    private DigitalChannel eh_digital_0_1;
    private DigitalChannel eh_digital_2_3;

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

        imu = hardwareMap.get(BNO055IMU.class, "ch_i2c_0");
        imu.initialize(parameters);

        ch_drive_lf = hardwareMap.get(DcMotor.class, "ch_drive_lf");
        ch_drive_lf.setMode(STOP_AND_RESET_ENCODER);
        ch_drive_lf.setMode(RUN_USING_ENCODER);
        ch_drive_lf.setDirection(FORWARD);

        ch_drive_rf = hardwareMap.get(DcMotor.class,"ch_drive_rf");
        ch_drive_rf.setMode(STOP_AND_RESET_ENCODER);
        ch_drive_rf.setMode(RUN_USING_ENCODER);
        ch_drive_rf.setDirection(REVERSE);

        ch_drive_lr = hardwareMap.get(DcMotor.class,"ch_drive_lr");
        ch_drive_lr.setMode(STOP_AND_RESET_ENCODER);
        ch_drive_lr.setMode(RUN_USING_ENCODER);
        ch_drive_lr.setDirection(FORWARD);

        ch_drive_rr = hardwareMap.get(DcMotor.class, "ch_drive_rr");
        ch_drive_rr.setMode(STOP_AND_RESET_ENCODER);
        ch_drive_rr.setMode(RUN_USING_ENCODER);
        ch_drive_rr.setDirection(REVERSE);

        eh_motor_0 = hardwareMap.get(DcMotor.class, "eh_motor_0");
        eh_motor_0.setMode(STOP_AND_RESET_ENCODER);
        eh_motor_0.setMode(RUN_USING_ENCODER);
        eh_motor_0.setDirection(FORWARD);

        eh_motor_1 = hardwareMap.get(DcMotor.class, "eh_motor_1");
        eh_motor_1.setMode(STOP_AND_RESET_ENCODER);
        eh_motor_1.setMode(RUN_USING_ENCODER);
        eh_motor_1.setDirection(FORWARD);

        eh_motor_2 = hardwareMap.get(DcMotor.class, "eh_motor_2");
        eh_motor_2.setMode(STOP_AND_RESET_ENCODER);
        eh_motor_2.setMode(RUN_USING_ENCODER);
        eh_motor_2.setDirection(FORWARD);

        eh_motor_3 = hardwareMap.get(DcMotor.class, "eh_motor_3");
        eh_motor_3.setMode(STOP_AND_RESET_ENCODER);
        eh_motor_3.setMode(RUN_USING_ENCODER);
        eh_motor_3.setDirection(FORWARD);

        eh_servo_0 = hardwareMap.get(Servo.class,"eh_servo_0");
        eh_servo_5 = hardwareMap.get(RevBlinkinLedDriver.class,"eh_servo_5");

        eh_digital_0_1 = hardwareMap.get(DigitalChannel.class, "eh_digital_0_1");
        eh_digital_2_3 = hardwareMap.get(DigitalChannel.class, "eh_digital_2_3");

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

    public void drive(double power, double turn) {
        if (opMode.isStopping()) return;

        double left = power + turn;
        double right = power - turn;

        double max = Math.max(Math.abs(left), Math.abs(right));

        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        ch_drive_lf.setPower(left);
        ch_drive_lr.setPower(left);
        ch_drive_rf.setPower(right);
        ch_drive_rr.setPower(right);
    }

    public void drive(double lf, double lr, double rf, double rr) {
        if (opMode.isStopping()) return;

        ch_drive_lf.setPower(lf);
        ch_drive_lr.setPower(lr);
        ch_drive_rf.setPower(rf);
        ch_drive_rr.setPower(rr);
    }

    public void setLights(RevBlinkinLedDriver.BlinkinPattern pattern) {
        eh_servo_5.setPattern(pattern == BLACK ? DEFAULT_COLOR : pattern);
    }

    private double getOffset(Recognition item) {
        // Linear Coordinates
        final double x1 = 140/*height*/, y1 = 14/*degrees*/;
        final double x2 = 254/*height*/, y2 = 9/*degrees*/;

        // Linear Equation: y(x) = y1 + ((y2 - y1) / (x2 - x1)) * (x - x1)
        return y1 + ((y2 - y1) / (x2 - x1)) * (item.getHeight() - x1);
    }

    public void setAttachmentMotorPower(double power0, double power1, double power2, double power3) {
        eh_motor_0.setPower(power0);
        eh_motor_1.setPower(power1);
        eh_motor_2.setPower(power2);
        eh_motor_3.setPower(power3);
    }

    public enum WobbleArmPosition {
        UP(0.10), DOWN(-0.10), STOP(0);

        public double power;

        WobbleArmPosition(double power) {
            this.power = power;
        }
    }

    public void wobbleArm(WobbleArmPosition position) {
        if ((position == WobbleArmPosition.UP && !eh_digital_0_1.getState()) ||
            (position == WobbleArmPosition.DOWN && !eh_digital_2_3.getState())) {
            eh_motor_2.setPower(0);
        } else {
            eh_motor_2.setPower(position.power);
        }
    }

    public enum WobbleLatchPosition {
        OPEN(0.25), CLOSED(-0.25);

        public double value;

        WobbleLatchPosition(double value) {
            this.value = value;
        }
    }

    public void wobbleLatch(WobbleLatchPosition position) {
        eh_servo_0.setPosition(position.value);
    }

    public void addTelemetry() {
        Telemetry telemetry = opMode.telemetry;

        orientation = getOrientation();

        telemetry.addData("Drive","%.2f Pow", opMode.gamepad2.left_stick_y);
        telemetry.addData("Turn","%.2f Pow", opMode.gamepad2.right_stick_x);
        telemetry.addData("Drive (LF)","%.2f Pow, %d Pos", ch_drive_lf.getPower(), ch_drive_lf.getCurrentPosition());
        telemetry.addData("Drive (LR)","%.2f Pow, %d Pos", ch_drive_lr.getPower(), ch_drive_lr.getCurrentPosition());
        telemetry.addData("Drive (RF)","%.2f Pow, %d Pos", ch_drive_rf.getPower(), ch_drive_rf.getCurrentPosition());
        telemetry.addData("Drive (RR)","%.2f Pow, %d Pos", ch_drive_rr.getPower(), ch_drive_rr.getCurrentPosition());
        telemetry.addData("Wobble Limit", eh_digital_0_1.getState());
        telemetry.addData("Wobble Touch", eh_digital_2_3.getState());
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
        ch_drive_lf.setMode(STOP_AND_RESET_ENCODER);
        ch_drive_lf.setMode(RUN_USING_ENCODER);
        ch_drive_lr.setMode(STOP_AND_RESET_ENCODER);
        ch_drive_lr.setMode(RUN_USING_ENCODER);
        ch_drive_rf.setMode(STOP_AND_RESET_ENCODER);
        ch_drive_rf.setMode(RUN_USING_ENCODER);
        ch_drive_rr.setMode(STOP_AND_RESET_ENCODER);
        ch_drive_rr.setMode(RUN_USING_ENCODER);
    }

    private double clamp(double min, double max, double value) {
        return value >= 0 ?
            Math.min(max, Math.max(min, value)) :
            Math.min(-min, Math.max(-max, value));
    }
}
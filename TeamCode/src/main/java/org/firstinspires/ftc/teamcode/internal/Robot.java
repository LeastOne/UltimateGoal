package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    private DcMotor left_front;
    private DcMotor left_rear;
    private DcMotor right_front;
    private DcMotor right_rear;

    private DcMotor eh_motor_0;
    private DcMotor eh_motor_1;
    private DcMotor eh_motor_2;
    private DcMotor eh_motor_3;

    private RevBlinkinLedDriver lights;

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

        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_front.setMode(STOP_AND_RESET_ENCODER);
        left_front.setMode(RUN_USING_ENCODER);
        left_front.setDirection(FORWARD);
        left_rear = hardwareMap.get(DcMotor.class,"left_rear");
        left_rear.setMode(STOP_AND_RESET_ENCODER);
        left_rear.setMode(RUN_USING_ENCODER);
        left_rear.setDirection(FORWARD);
        right_front = hardwareMap.get(DcMotor.class,"right_front");
        right_front.setMode(STOP_AND_RESET_ENCODER);
        right_front.setMode(RUN_USING_ENCODER);
        right_front.setDirection(REVERSE);
        right_rear = hardwareMap.get(DcMotor.class, "right_rear");
        right_rear.setMode(STOP_AND_RESET_ENCODER);
        right_rear.setMode(RUN_USING_ENCODER);
        right_rear.setDirection(REVERSE);

        eh_motor_0 = hardwareMap.get(DcMotor.class, "eh_motor_0");
        eh_motor_1 = hardwareMap.get(DcMotor.class, "eh_motor_1");
        eh_motor_2 = hardwareMap.get(DcMotor.class, "eh_motor_2");
        eh_motor_3 = hardwareMap.get(DcMotor.class, "eh_motor_3");

        lights = hardwareMap.get(RevBlinkinLedDriver.class,"lights");

        webcamName = hardwareMap.get(WebcamName.class,"Webcam 1");
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId","id",hardwareMap.appContext.getPackageName());

        visionThread = new VisionThread(opMode,this);
        visionThread.start();

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
        if (!opMode.isContinuing()) return;

        double left = power + turn;
        double right = power - turn;

        double max = Math.max(Math.abs(left), Math.abs(right));

        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        left_front.setPower(left);
        left_rear.setPower(left);
        right_front.setPower(right);
        right_rear.setPower(right);
    }

    public void drive(double lf, double lr, double rf, double rr) {
        if (!opMode.isContinuing()) return;

        left_front.setPower(lf);
        left_rear.setPower(lr);
        right_front.setPower(rf);
        right_rear.setPower(rr);
    }

    public void drive(double power, double heading, double inches) {
        if (!opMode.isContinuing()) return;

        turn(power, heading);

        int targetPosition = (int)(inches * TICKS_PER_INCH);
        int position = 0;

        double remainder, turn;

        while (opMode.isContinuing() && targetPosition - position > 0) {
            remainder = getRemainderLeftToTurn(heading);
            power = clamp(0.2, power, (1 - (double)position / targetPosition) * TICKS_PER_INCH * 12);
            turn = remainder / 45;
            drive(power, turn);

            position = (
                Math.abs(left_front.getCurrentPosition()) +
                Math.abs(left_rear.getCurrentPosition()) +
                Math.abs(right_front.getCurrentPosition()) +
                Math.abs(right_rear.getCurrentPosition())
            ) / 4;
        }

        this.drive(0,0);
    }

    public void turn(double power, double heading) {
        if (!opMode.isContinuing()) return;

        power = Math.abs(power);

        double remainder, turn;

        do {
            remainder = getRemainderLeftToTurn(heading);
            turn = clamp(0.2, power, remainder / 45 * power);
            drive(0, turn);
        } while (opMode.isContinuing() && (remainder < -1 || remainder > 1));

        drive(0,0);
    }

    public void setLights (RevBlinkinLedDriver.BlinkinPattern pattern) {
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
        eh_motor_0.setPower(power0);
        eh_motor_1.setPower(power1);
        eh_motor_2.setPower(power2);
        eh_motor_3.setPower(power3);
    }

    public void addTelemetry(){
        Telemetry telemetry = opMode.telemetry;

        orientation = getOrientation();

        telemetry.addData("Drive","%.2f Pow", opMode.gamepad2.left_stick_y);
        telemetry.addData("Turn","%.2f Pow", opMode.gamepad2.right_stick_x);
        telemetry.addData("Drive (LF)","%.2f Pow, %d Pos", left_front.getPower(), left_front.getCurrentPosition());
        telemetry.addData("Drive (LR)","%.2f Pow, %d Pos", left_rear.getPower(), left_rear.getCurrentPosition());
        telemetry.addData("Drive (RF)","%.2f Pow, %d Pos", right_front.getPower(), right_front.getCurrentPosition());
        telemetry.addData("Drive (RR)","%.2f Pow, %d Pos", right_rear.getPower(), right_rear.getCurrentPosition());
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
                telemetry.addData(" label", recognition.getLabel());
                telemetry.addData("  left,top", "%.3f , %.3f", recognition.getLeft(), recognition.getTop());
                telemetry.addData("  right,bottom", "%.3f , %.3f", recognition.getRight(), recognition.getBottom());
                telemetry.addData("  height,width", "%.3f , %.3f", recognition.getHeight(), recognition.getWidth());
                telemetry.addData("  angle", "%.3f", recognition.estimateAngleToObject(DEGREES));
                telemetry.addData("  offset", "%.3f", getOffset(recognition));
                telemetry.addData("  heading", "%.3f", recognition.estimateAngleToObject(DEGREES) + getOffset(recognition));
                telemetry.addData("  area", "%.3f", recognition.getWidth() * recognition.getHeight());
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
        left_front.setMode(STOP_AND_RESET_ENCODER);
        left_front.setMode(RUN_USING_ENCODER);
        left_rear.setMode(STOP_AND_RESET_ENCODER);
        left_rear.setMode(RUN_USING_ENCODER);
        right_front.setMode(STOP_AND_RESET_ENCODER);
        right_front.setMode(RUN_USING_ENCODER);
        right_rear.setMode(STOP_AND_RESET_ENCODER);
        right_rear.setMode(RUN_USING_ENCODER);
    }

    private double clamp(double min, double max, double value) {
        return value >= 0 ?
            Math.min(max, Math.max(min, value)) :
            Math.min(-min, Math.max(-max, value));
    }
}

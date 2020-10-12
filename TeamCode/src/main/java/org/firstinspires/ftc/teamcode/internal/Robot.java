package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
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
import static org.firstinspires.ftc.teamcode.internal.Robot.ClawPosition.CLOSE;
import static org.firstinspires.ftc.teamcode.internal.Robot.ClawPosition.OPEN;

public class Robot {
    private static final double INCHES_PER_ROTATION = 3.95 * Math.PI;
    private static final double TICKS_PER_INCH = 1120 / INCHES_PER_ROTATION;

    private static RevBlinkinLedDriver.BlinkinPattern DEFAULT_COLOR = GRAY;
    private static RevBlinkinLedDriver.BlinkinPattern CALIBRATE_COLOR = RAINBOW_LAVA_PALETTE;
    private static RevBlinkinLedDriver.BlinkinPattern READY_COLOR = HEARTBEAT_GRAY;
    private static RevBlinkinLedDriver.BlinkinPattern SEARCHING_COLOR = LIGHT_CHASE_GRAY;
    private static RevBlinkinLedDriver.BlinkinPattern PICKUP_COLOR = GREEN;
    private static RevBlinkinLedDriver.BlinkinPattern TARGET_COLOR = YELLOW;

    private OpMode opMode;

    private BNO055IMU imu;

    private DcMotor left_front;
    private DcMotor left_rear;
    private DcMotor right_front;
    private DcMotor right_rear;

    //Creating motors for mecanum wheels
    private DcMotor mec_lf;
    private DcMotor mec_lb;
    private DcMotor mec_rf;
    private DcMotor mec_rb;

    private DcMotor slide;
    private DigitalChannel slide_limit_front;
    private DigitalChannel slide_limit_rear;

    private DcMotor tilt;
    private DigitalChannel tilt_limit;
    private ModernRoboticsI2cCompassSensor tilt_accelerometer;
    private boolean tiltIsBusy = false;

    private DcMotor lift;

    private Servo claw_left;
    private Servo claw_right;

    private Servo stick;

    private RevBlinkinLedDriver lights;

    private VisionThread visionThread;

    public WebcamName webcamName;
    public int cameraMonitorViewId;
    public int tfodMonitorViewId;

    public boolean targetVisible = false;
    public Position position = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    public Orientation orientation = new Orientation();

    public boolean skystoneVisible = false;
    public Position skystonePosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    public Orientation skystoneOrientation = new Orientation();

    public double skystoneHeight;

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

        slide = hardwareMap.get(DcMotor.class,"slide");
        slide.setMode(STOP_AND_RESET_ENCODER);
        slide.setMode(RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(BRAKE);
        slide.setDirection(REVERSE);
        slide_limit_front = hardwareMap.get(DigitalChannel.class, "slide_limit_front");
        slide_limit_front.setMode(INPUT);
        slide_limit_rear = hardwareMap.get(DigitalChannel.class, "slide_limit_rear");
        slide_limit_rear.setMode(INPUT);

        tilt = hardwareMap.get(DcMotor.class, "tilt");
        tilt.setMode(RUN_USING_ENCODER);
        tilt.setZeroPowerBehavior(BRAKE);
        tilt_limit = hardwareMap.get(DigitalChannel.class, "tilt_limit");
        tilt_accelerometer = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "tilt_accelerometer");

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(STOP_AND_RESET_ENCODER);
        lift.setMode(RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(BRAKE);
        lift.setDirection(REVERSE);

        claw_left = hardwareMap.get(Servo.class, "claw_left");
        claw_right = hardwareMap.get(Servo.class, "claw_right");

        stick = hardwareMap.get(Servo.class, "stick");

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

//        double maxChange = 0.33;
//        double leftCurrent = left_front.getPower();
//        double rightCurrent = right_front.getPower();
//        if (left != 0) left = leftCurrent + clamp(-maxChange, maxChange, left - leftCurrent);
//        if (right != 0) right = rightCurrent + clamp(-maxChange, maxChange, right - rightCurrent);

        left_front.setPower(left);
        left_rear.setPower(left);
        right_front.setPower(right);
        right_rear.setPower(right);
    }

    public void drive(double lf, double lb, double rf, double rb){

        if(mecanumMode){
            mec_lf.setPower(lf);
            mec_lb.setPower(lb);
            mec_rf.setPower(rf);
            mec_rb.setPower(rb);
        }
        else{
            if (!opMode.isContinuing()) return;

            double left = lf;
            double right = rf;

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

    }

    public void drive(double power, double heading, double inches) {
        if (!opMode.isContinuing()) return;

        turn(power, heading);

        resetEncoders();

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

    public void lift(double power) {
        //if (tilt_accelerometer.getAcceleration().yAccel > TILTED.accel) return;

        int minPos = 0;
        int maxPos = 5600;

        int position = lift.getCurrentPosition();

        if((power > 0 && position < maxPos) || (power < 0 && position > minPos)){
            lift.setPower(power);
        } else {
            lift.setPower(0);
        }
    }

    public enum ClawPosition { OPEN, CLOSE }

    public void claw(ClawPosition position) {
        if (!opMode.isContinuing()) return;
        claw_left.setPosition(position == OPEN ? 1 : 0.15);
        claw_right.setPosition(position == CLOSE ? 0.85 : 0.25 );
        sleep(0.25);
    }

    public void stickToggle(){
        if (!opMode.isContinuing()) return;
        double power = stick.getPosition() > 0.5 ? 0 : 1;
        stick.setPosition(power);
        sleep(0.25);
    }

    public void setLights (RevBlinkinLedDriver.BlinkinPattern pattern){
        lights.setPattern(pattern == BLACK ? DEFAULT_COLOR : pattern);
    }

    public Recognition findNearestStone(Boolean lookingForSkystone){
        Recognition nearestRecognition = null;

        String searchString = lookingForSkystone ? "skystone" : "stone";

        for (Recognition recognition : recognitions) {
            if (recognition.getLabel().toLowerCase().contains(searchString) && (
                nearestRecognition == null ||
                recognition.getHeight() > nearestRecognition.getHeight()
            )) {
                nearestRecognition = recognition;
            }
        }

        return nearestRecognition;
    }

    public boolean isSkystoneVisible() {
        return skystoneVisible || (System.nanoTime() - skystonePosition.acquisitionTime < 1000000000);
    }

    Boolean stonePickUp = false;

    public void pickUpStone(boolean lookingForStone) {
        if (!opMode.isContinuing()) return;

        stonePickUp = true;
        setLights(SEARCHING_COLOR);
        double power = 0.25;

        Recognition stone = null;

        while (opMode.isContinuing()) {
            stone = findNearestStone(lookingForStone);
            if (stone != null) break;
        }

        turn(power, getOrientation().firstAngle + stone.estimateAngleToObject(DEGREES) + getOffset(stone));

        while (opMode.isContinuing()) {
            stone = findNearestStone(lookingForStone);
            if (stone == null) drive(0,0);
            else if (stone.getHeight() > 250) break;
            else drive(power, (stone.estimateAngleToObject(DEGREES) + getOffset(stone)) / -45);
        }

        drive(0,0);

        claw(OPEN);
        drive(power, getOrientation().firstAngle,12);
        claw(CLOSE);

        setLights(PICKUP_COLOR);
        sleep(0.5);
        setLights(DEFAULT_COLOR);
        stonePickUp = false;
    }

    private double getOffset(Recognition stone) {
        // Linear Coordinates
        final double x1 = 140/*height*/, y1 = 14/*degrees*/;
        final double x2 = 254/*height*/, y2 = 9/*degrees*/;

        // Linear Equation: y(x) = y1 + ((y2 - y1) / (x2 - x1)) * (x - x1)
        return y1 + ((y2 - y1) / (x2 - x1)) * (stone.getHeight() - x1);
    }

    //more than 10 is 1
    //between -10 and 10 is 2
    //less than -10 is 3

    public void pickUpSkystone(){
        if (!opMode.isContinuing()) return;

        stonePickUp = true;
        setLights(SEARCHING_COLOR);

        double power = 0.25;

        while(opMode.isContinuing() && !isSkystoneVisible());

        double targetAngle = Math.toDegrees(Math.atan2(-skystonePosition.y, -skystonePosition.x));
        double currentAngle = skystoneOrientation.thirdAngle;
        double heading = getOrientation().firstAngle + currentAngle;

        if(heading <= -9 ){heading = -18;}
        else if(heading >=9){heading = 18;}
        else{heading = 0;}

        double inches = Math.sqrt(
            Math.pow(skystonePosition.x, 2) +
            Math.pow(skystonePosition.y, 2)
        );

        claw(OPEN);
        drive(power, heading, inches - 6);
        claw(CLOSE);

        setLights(PICKUP_COLOR);
        sleep(0.5);
        setLights(DEFAULT_COLOR);
        stonePickUp = false;
    }

    public void addTelemetry(){
        Telemetry telemetry = opMode.telemetry;

        orientation = getOrientation();

        telemetry.addData("Drive","%.2f Pow", opMode.gamepad2.left_stick_y);
        telemetry.addData("Turn","%.2f Pow", opMode.gamepad2.right_stick_x);
        telemetry.addData("Drive (LF)","%.2f Pow, %d Pos", left_front.getPower(), left_front.getCurrentPosition());
        telemetry.addData("Drive (LF)","%.2f Pow, %d Pos", left_front.getPower(), left_front.getCurrentPosition());
        telemetry.addData("Drive (LF)","%.2f Pow, %d Pos", left_front.getPower(), left_front.getCurrentPosition());


        telemetry.addData("Drive (LF)","%.2f Pow, %d Pos", left_front.getPower(), left_front.getCurrentPosition());
        telemetry.addData("Drive (LR)","%.2f Pow, %d Pos", left_rear.getPower(), left_rear.getCurrentPosition());
        telemetry.addData("Drive (RF)","%.2f Pow, %d Pos", right_front.getPower(), right_front.getCurrentPosition());
        telemetry.addData("Drive (RR)","%.2f Pow, %d Pos", right_rear.getPower(), right_rear.getCurrentPosition());
        telemetry.addData("Slide","%.2f Pow, %d Pos", slide.getPower(), slide.getCurrentPosition());
        telemetry.addData("Slide Limit Front", slide_limit_front.getState());
        telemetry.addData("Slide Limit Rear", slide_limit_rear.getState());
        telemetry.addData("Tilt","%.2f Pow, %d Pos", tilt.getPower(), tilt.getCurrentPosition());
        telemetry.addData("Tilt Limit", tilt_limit.getState());
        telemetry.addData("Tilt Accelerometer", tilt_accelerometer.getAcceleration());
        telemetry.addData("Lift","%.2f Pow, %d Pos", lift.getPower(), lift.getCurrentPosition());
        telemetry.addData("Target Visible", targetVisible);
        telemetry.addData("Position (in)", position);
        telemetry.addData("Orientation", orientation);
        telemetry.addData("Skystone Visible", skystoneVisible);
        telemetry.addData("Skystone Position (in)", skystonePosition);
        telemetry.addData("Skystone Orientation", skystoneOrientation);
        telemetry.addData("PickUpStone running", stonePickUp);

        telemetry.addLine();


        Boolean stoneVisible = false;

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

                if (recognition.getLabel().toLowerCase().contains("stone")) {
                    stoneVisible = true;
                }
            }
        }

        if(!stonePickUp) {
            setLights(stoneVisible ? TARGET_COLOR : DEFAULT_COLOR);
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

    private void sleep(double seconds) {
        try {
            Thread.sleep((long)(1000 * seconds));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}

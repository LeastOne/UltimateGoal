package org.firstinspires.ftc.teamcode.internal;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VisionThread extends Thread {
    @SuppressWarnings("SpellCheckingInspection")
    private static final String VUFORIA_KEY = "ARamgZ3/////AAABmb/8COrlYkzjkpr2MsKcDJpcbuv0xKXvLyTMSLb4kAcVeN8A560evWWup58hT12DOf5dGmaTtmR9OZaXZLgR41YJOte87AcvnY409wWEO3qp1y8iMpzVKDPZl6vXN+C9+8EnwojYg4ZcNsbCYQsu79Ghetb/Kji0CYUG/3HEvNkbd669uiL6zFWW+zllIh9x0ceLZLxKqIVGQGpamxt26UU8wYO2FqVoSo+DIZcofulZkv/MGNkAXdisHuclym2IjfW8yAEgLcJOgW2PKGNkLMj7lPYMNhK/5cpDM/zsTSW+SdPACaUzvmfK+h5iqreD569EpTk2P5tnZeo8e5hnbzWsA1CGGtvA5Z/ZhKjRjtqk";
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch; // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59; // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private OpMode opMode;
    private Robot robot;

    private TFObjectDetector tfod;

    public VisionThread(OpMode opMode, Robot robot){
        this.opMode = opMode;
        this.robot = robot;
    }

    @Override public void run() {
        try {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(robot.cameraMonitorViewId);
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = robot.webcamName;

            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
            VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
            blueTowerGoalTarget.setName("Blue Tower Goal Target");
            VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
            redTowerGoalTarget.setName("Red Tower Goal Target");
            VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
            redAllianceTarget.setName("Red Alliance Target");
            VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
            blueAllianceTarget.setName("Blue Alliance Target");
            VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
            frontWallTarget.setName("Front Wall Target");


            // For convenience, gather together all the trackable objects in one easily-iterable collection */
            List<VuforiaTrackable> allTrackables = new ArrayList<>(targetsUltimateGoal);

            // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
            // Rotated it to to face forward, and raised it to sit on the ground correctly.
            // This can be used for generic target-centric approach algorithms
            redAllianceTarget.setLocation(OpenGLMatrix
                    .translation(0, -halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            blueAllianceTarget.setLocation(OpenGLMatrix
                    .translation(0, halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
            frontWallTarget.setLocation(OpenGLMatrix
                    .translation(-halfField, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

            // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
            blueTowerGoalTarget.setLocation(OpenGLMatrix
                    .translation(halfField, quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
            redTowerGoalTarget.setLocation(OpenGLMatrix
                    .translation(halfField, -quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            // Next, translate the camera lens to where it is on the robot.
            // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
            final float CAMERA_FORWARD_DISPLACEMENT = 8.125f * mmPerInch; // eg: Camera is 4 Inches in front of robot-center
            final float CAMERA_VERTICAL_DISPLACEMENT = 5.25f * mmPerInch; // eg: Camera is 8 Inches above ground
            final float CAMERA_LEFT_DISPLACEMENT = -5.25f;     // eg: Camera is ON the robot's center line

            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, -90, 0, -75));

            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(robot.webcamName, robotFromCamera);
            }

            targetsUltimateGoal.activate();

            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(robot.tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
            tfod.activate();

            opMode.waitForStart();

            while (opMode.isActive()) {
                // check all the trackable targets to see which one (if any) is visible.
                boolean targetVisible = false;

                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();

                        if (robotLocationTransform != null) {
                            VectorF translation = robotLocationTransform.getTranslation();

                            Position position = new Position(
                                DistanceUnit.INCH,
                                translation.get(0) / mmPerInch,
                                translation.get(1) / mmPerInch,
                                translation.get(2) / mmPerInch,
                                System.nanoTime()
                            );

                            Orientation orientation = Orientation.getOrientation(robotLocationTransform, EXTRINSIC, XYZ, DEGREES);

                            if (!trackable.getName().equals("Stone Target")) {
                                targetVisible = true;
                                lastLocation = robotLocationTransform;
                                robot.position = position;
                                robot.orientation = orientation;
                            } else {
                                robot.itemPosition = position;
                                robot.itemOrientation = orientation;
                            }
                        }
                    }
                }

                robot.navigationTargetVisible = targetVisible;

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    robot.recognitions = updatedRecognitions;
                }
            }

            targetsUltimateGoal.deactivate();
        } catch (Exception e) {
            robot.error = e.toString();
        }
    }
}
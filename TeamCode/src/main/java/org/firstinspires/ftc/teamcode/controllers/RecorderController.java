package org.firstinspires.ftc.teamcode.controllers;

import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.DISABLED;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecordSelect.NEXT;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecordSelect.PREV;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.IDLE;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.RECORDING;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.REPLAYING;

public class RecorderController extends RobotController {
    public enum RecorderState {
        DISABLED,
        IDLE,
        RECORDING,
        REPLAYING
    }

    public RecorderState state = DISABLED;

    public static class RecorderSettings {
        public String redFileName;
        public String currentFileName;
        public String blueFileName;
    }

    public enum RecordSelect {
        PREV(-1),
        NEXT(1);

        public int value;

        RecordSelect(int value){
            this.value = value;
        }
    }

    public static class RecorderSnapshot {
        public long timestamp = System.currentTimeMillis();
        public Gamepad[] gamepads = new Gamepad[] { new Gamepad(), new Gamepad() };
        public int[] motorPositions;
    }

    private static String BASE_DIR = "recordings/";

    private long replayStart;
    private long recordStart;

    private RecorderSettings settings = new RecorderSettings();

    private List<RecorderSnapshot> snapshots = new ArrayList<>();

    private boolean isSlept = true;
    private boolean isSleepSynced = true;
    private boolean isPosSynced = false;

    public RecorderController(OpMode opMode) {
        super(opMode);
        loadSettings();
    }

    @Override
    public void execute() {
        //Start, Back, and not at rest to start recording mode
        if (gamepad1.start && gamepad1.back && !gamepad1.atRest()) state =  IDLE;
        if (state == DISABLED) return;

        switch(state) {
            case IDLE:
                if (gamepad1.start && gamepad1.y) enterRecording();
                if (gamepad1.y && !gamepad1.atRest()) enterReplaying(settings.currentFileName);
                if (gamepad1.x && !gamepad1.atRest()) enterReplaying(settings.blueFileName);
                if (gamepad1.b && !gamepad1.atRest()) enterReplaying(settings.redFileName);
                if (gamepad1.y && gamepad1.dpad_left) settings.currentFileName = selectRecording(settings.currentFileName, PREV);
                if (gamepad1.y && gamepad1.dpad_right) settings.currentFileName = selectRecording(settings.currentFileName, NEXT);
                if (gamepad1.x && gamepad1.dpad_left) settings.blueFileName = selectRecording(settings.blueFileName, PREV);
                if (gamepad1.x && gamepad1.dpad_right) settings.blueFileName = selectRecording(settings.blueFileName, NEXT);
                if (gamepad1.b && gamepad1.dpad_left) settings.redFileName = selectRecording(settings.redFileName, PREV);
                if (gamepad1.b && gamepad1.dpad_right) settings.redFileName = selectRecording(settings.redFileName, NEXT);
                if (gamepad1.back || gamepad1.dpad_right || gamepad1.dpad_left) saveSettings();
                break;
            case RECORDING:
                if (gamepad1.back) enterIdle();
                else record();
                break;
            case REPLAYING:
                if (gamepad1.back) enterIdle();
                else if (snapshots.size() != 0) replay();
                else enterIdle();
                break;
        }

        telemetry.addData("Recording State", state);
        telemetry.addData("Gamepad States", snapshots == null ? 0 : snapshots.size());
        telemetry.addData("Current Recording", settings.currentFileName);
        telemetry.addData("Blue Recording", settings.blueFileName);
        telemetry.addData("Red Recording", settings.redFileName);
        telemetry.addLine();
    }

    private void record() {
        if (snapshots.isEmpty() && gamepad1.atRest() && gamepad2.atRest()) return;

        RecorderSnapshot snapshot = new RecorderSnapshot();

        try {
            snapshot.gamepads[0].copy(gamepad1);
            snapshot.gamepads[1].copy(gamepad2);
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (isSlept) opMode.sleep(50);

        snapshot.motorPositions = robot.getMotorPositions();

        snapshots.add(snapshot);
    }

    private void replay() {
        RecorderSnapshot snapshot = snapshots.remove(0);

        if (isPosSynced) robot.setMotorPositions(snapshot.motorPositions);

        if (isSleepSynced) {
            long replayDuration = System.currentTimeMillis() - replayStart;
            long recordDuration = snapshot.timestamp - recordStart;
            long sleep = recordDuration - replayDuration;

            if (sleep > 0) opMode.sleep(sleep);
        }

        try {
            gamepad1.copy(snapshot.gamepads[0]);
            gamepad2.copy(snapshot.gamepads[1]);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void enterIdle() {
        if (state == RECORDING) saveRecording(settings.currentFileName);
        state = IDLE;
        robot.resetDriveEncoders(RUN_USING_ENCODER);
    }

    private void enterRecording() {
        snapshots.clear();
        state = RECORDING;
        robot.resetDriveEncoders(RUN_USING_ENCODER);
        SimpleDateFormat sim = new SimpleDateFormat("yyyy-MM-dd--HH-mm-ss", Locale.US);
        settings.currentFileName = sim.format(new Date()) + ".json";
    }

    private void enterReplaying(String fileName) {
        state = REPLAYING;
        robot.resetDriveEncoders(isPosSynced ? RUN_TO_POSITION : RUN_USING_ENCODER);
        loadRecording(fileName);
        replayStart = System.currentTimeMillis();
        recordStart = snapshots.get(0).timestamp;
    }

    private String selectRecording(String filename, RecordSelect select) {
        opMode.sleep(250);

        File directory = AppUtil.getInstance().getSettingsFile(BASE_DIR);
        File[] files = directory.listFiles();

        if (files == null) return filename;

        int index = 0;

        for (; index < files.length; index++)
            if (files[index].getName().equals(filename))
                break;

        index += select.value;

        return index >= 0 && index < files.length ? files[index].getName() : filename;
    }

    private void saveSettings() {
        String json = SimpleGson.getInstance().toJson(settings);
        File file = AppUtil.getInstance().getSettingsFile(BASE_DIR + "settings.json");
        ReadWriteFile.writeFile(file, json);
    }

    private void loadSettings() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(BASE_DIR + "settings.json");
            if (!file.exists()) return;
            String json = ReadWriteFile.readFileOrThrow(file);
            settings = SimpleGson.getInstance().fromJson(json, new TypeToken<RecorderSettings>() {}.getType());
        } catch (Exception e) {
            telemetry.addData("Error", "An error occurred attempting to load the settings" + e.toString());
        }
    }

    private void saveRecording(String fileName) {
        String json = SimpleGson.getInstance().toJson(snapshots.toArray());
        File file = AppUtil.getInstance().getSettingsFile(BASE_DIR + fileName);
        ReadWriteFile.writeFile(file, json);
    }

    private void loadRecording(String fileName) {
        try {
            File file = AppUtil.getInstance().getSettingsFile(BASE_DIR + fileName);
            if (!file.exists()) return;
            String json = ReadWriteFile.readFileOrThrow(file);
            snapshots = SimpleGson.getInstance().fromJson(json, new TypeToken<List<RecorderSnapshot>>(){}.getType());
        } catch(Exception e) {
            telemetry.addData("Error","An error occurred attempting to load the recording data" + e.toString());
        }
    }
}
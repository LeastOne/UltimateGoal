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

import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.DISABLED;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecordSelect.NEXT;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecordSelect.PREV;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.IDLE;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.RECORDING;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.REPLAYING;

public class RecorderController extends RobotController {
    public enum RecorderState{
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

    public enum RecordSelect{
        PREV(-1),
        NEXT(1);

        public int value;

        RecordSelect(int value){
            this.value = value;
        }
    }

    private static String BASE_DIR = "recordings/";

    private RecorderSettings settings = new RecorderSettings();

    private List<Gamepad> gamepads = new ArrayList<>();

    public RecorderController(OpMode opMode) {
        super(opMode);
        loadSettings();
    }

    @Override
    public void execute() {
        //Start, Back, and not at rest to start recording mode
        if(gamepad1.start && gamepad1.back && !gamepad1.atRest()) state =  IDLE;

        if(state == DISABLED) return;

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
                else if (gamepads.size() != 0) replay();
                else enterIdle();
                break;
        }

        telemetry.addData("Recording State", state);
        telemetry.addData("Gamepad States", gamepads == null ? 0 : gamepads.size());
        telemetry.addData("Current Recording", settings.currentFileName);
        telemetry.addData("Blue Recording", settings.blueFileName);
        telemetry.addData("Red Recording", settings.redFileName);
        telemetry.addLine();
    }

    private void record() {
        Gamepad gamepad1copy = new Gamepad();
        Gamepad gamepad2copy = new Gamepad();

        if (gamepads.isEmpty() && gamepad1.atRest() && gamepad2.atRest()) return;

        try {
            gamepad1copy.copy(gamepad1);
            gamepad2copy.copy(gamepad2);
        } catch (Exception e) {
            e.printStackTrace();
        }

        gamepads.add(gamepad1copy);
        gamepads.add(gamepad2copy);
    }

    private void replay() {
        Gamepad gamepad1Copy = gamepads.remove(0);
        Gamepad gamepad2Copy = gamepads.remove(0);

        try {
            gamepad1.copy(gamepad1Copy);
            gamepad2.copy(gamepad2Copy);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void enterIdle() {
        if (state == RECORDING) saveRecording(settings.currentFileName);
        state = IDLE;
    }

    private void enterRecording() {
        gamepads.clear();
        state = RECORDING;
        SimpleDateFormat sim = new SimpleDateFormat("yyyy-MM-dd--hh-mm-ss", Locale.US);
        settings.currentFileName = sim.format(new Date()) + ".json";
    }

    private void enterReplaying(String fileName) {
        state = REPLAYING;
        loadRecording(fileName);
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
        String json = SimpleGson.getInstance().toJson(gamepads.toArray());
        File file = AppUtil.getInstance().getSettingsFile(BASE_DIR + fileName);
        ReadWriteFile.writeFile(file, json);
    }

    private void loadRecording(String fileName) {
        try {
            File file = AppUtil.getInstance().getSettingsFile(BASE_DIR + fileName);
            if (!file.exists()) return;
            String json = ReadWriteFile.readFileOrThrow(file);
            gamepads = SimpleGson.getInstance().fromJson(json, new TypeToken<List<Gamepad>>(){}.getType());
        } catch(Exception e) {
            telemetry.addData("Error","An error occurred attempting to load the recording data" + e.toString());
        }
    }
}
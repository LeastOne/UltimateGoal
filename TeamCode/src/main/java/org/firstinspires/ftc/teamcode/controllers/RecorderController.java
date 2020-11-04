package org.firstinspires.ftc.teamcode.controllers;

import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

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

    public class RecorderSettings {
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

    private RecorderSettings settings = new RecorderSettings();

    private RecorderState state = DISABLED;

    private List<Gamepad> gamepads = null;

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
                if (gamepad1.dpad_right || gamepad1.dpad_left) saveSettings();
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
        telemetry.addData("Gamepad States",gamepads == null ? 0 : gamepads.size());
        telemetry.addData("current recording",settings.currentFileName);
        telemetry.addData("blue recording",settings.blueFileName);
        telemetry.addData("red recording",settings.redFileName);
        telemetry.addLine();
    }

    private void record() {
        Gamepad gamepad1copy = new Gamepad();
        Gamepad gamepad2copy = new Gamepad();

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
        if (state == RECORDING) saveRecording();
        state = IDLE;
    }

    private void enterRecording() {
        gamepads = new ArrayList<>();
        state = RECORDING;
        SimpleDateFormat sim = new SimpleDateFormat("yyyy-MM-dd--hh-mm-ss");
        settings.currentFileName = "recordings/" + sim.format(new Date()) + ".json";
    }

    private void enterReplaying( String fileName) {
        state = REPLAYING;
        loadRecording(fileName);
    }

    private String selectRecording(String filename, RecordSelect select) {
        File directory = AppUtil.getInstance().getSettingsFile("recordings/");
        File[] files = directory.listFiles();
        int index = 0;
        for(;index<files.length;index++){
            if(files[index].getName().equals(filename))break;
        }
        index+=select.value;
        if(0<=index&&index<files.length)return files[index].getName();
        return files[index].getName();
    }

    private void saveSettings() {
        String json = SimpleGson.getInstance().toJson(settings);
        File file = AppUtil.getInstance().getSettingsFile("recordings/settings.json");
        ReadWriteFile.writeFile(file,json);
    }

    private void loadSettings() {
        try {
            File file = AppUtil.getInstance().getSettingsFile("recordings/settings.json");
            String json = ReadWriteFile.readFileOrThrow(file);
            settings = SimpleGson.getInstance().fromJson(json, new TypeToken<RecorderSettings>() {}.getType());
        } catch (Exception e) {
            telemetry.addData("Error", "An error occurred attempting to load the settings" + e.toString());
        }
    }
    private void saveRecording() {
        String json = SimpleGson.getInstance().toJson(gamepads.toArray());
        File file = AppUtil.getInstance().getSettingsFile(settings.currentFileName);
        ReadWriteFile.writeFile(file,json);
    }

    private void loadRecording(String fileName ) {
        try {
            File file = AppUtil.getInstance().getSettingsFile(fileName);
            String json = ReadWriteFile.readFileOrThrow(file);
            gamepads = SimpleGson.getInstance().fromJson(json, new TypeToken<List<Gamepad>>(){}.getType());
        } catch(Exception e) {
            telemetry.addData("Error","An error occurred attempting to load the recording data" + e.toString());
        }
    }
}
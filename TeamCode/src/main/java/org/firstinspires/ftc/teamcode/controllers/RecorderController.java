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

import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.IDLE;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.RECORDING;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.REPLAYING;

public class RecorderController extends RobotController {
    public enum RecorderState{
        IDLE,
        RECORDING,
        REPLAYING
    }

    public class RecorderSettings {

    }

    public enum RecordSelect{
        PREV,
        NEXT
    }

    private String currentFileName;
    private String redFileName;
    private String blueFileName;

    private RecorderState state = IDLE;

    private List<Gamepad> gamepads = null;

    public RecorderController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        switch(state) {
            case IDLE:
                if (gamepad1.start && gamepad1.y) enterRecording();
                if (gamepad1.y && !gamepad1.atRest()) enterReplaying();
                if (gamepad1.x && !gamepad1.atRest()) enterReplaying();
                if (gamepad1.b && !gamepad1.atRest()) enterReplaying();
                if (gamepad1.x && gamepad1.dpad_left) blueFileName = selectRecording(blueFileName, RecordSelect.PREV);
                if (gamepad1.x && gamepad1.dpad_right) blueFileName = selectRecording(blueFileName, RecordSelect.NEXT);
                if (gamepad1.b && gamepad1.dpad_left) redFileName = selectRecording(redFileName, RecordSelect.PREV);
                if (gamepad1.b && gamepad1.dpad_right) redFileName = selectRecording(redFileName, RecordSelect.NEXT);
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
        if (state == RECORDING) save();
        state = IDLE;
    }

    private void enterRecording() {
        gamepads = new ArrayList<>();
        state = RECORDING;
        SimpleDateFormat sim = new SimpleDateFormat("yyyy-MM-dd--hh-mm-ss");
        currentFileName = "recordings/" + sim.format(new Date()) + ".json";
    }

    private void enterReplaying() {
        //File directory = new File("./");
        //File[] files = directory.listFiles();
        state = REPLAYING;
        load();
    }

    private String selectRecording(String filename, RecordSelect select) {
        return null;
    }

    private void save() {
        String json = SimpleGson.getInstance().toJson(gamepads.toArray());
        File file = AppUtil.getInstance().getSettingsFile(currentFileName);
        ReadWriteFile.writeFile(file,json);
    }

    private void load() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(currentFileName);
            String json = ReadWriteFile.readFileOrThrow(file);
            gamepads = SimpleGson.getInstance().fromJson(json, new TypeToken<List<Gamepad>>(){}.getType());
        } catch(Exception e) {
            telemetry.addData("Error","An error occurred attempting to load the playback data" + e.toString());
        }
    }
}
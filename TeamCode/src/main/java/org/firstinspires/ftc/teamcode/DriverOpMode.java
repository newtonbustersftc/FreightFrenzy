package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.io.File;
import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.AutonomousOptions.DELAY_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.DELIVER_ROUTE_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.FOUNDATION_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.PARKING_ONLY_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.PARKING_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.STONE_PREF;

@TeleOp(name="Newton DriverOpMode", group="Main")
//@Disabledxxs
public class DriverOpMode extends OpMode {
    RobotHardware robotHardware;
    RobotNavigator navigator;
    RobotProfile robotProfile;
    DriverOptions driverOptions;

    double gyroCurrAngle;
    double gyroAngleOffset;

    boolean fieldMode;
    boolean xAlreadyPressed = false;
    boolean aAlreadyPressed = false;
    boolean bAlreadyPressed = false;
    boolean yAlreadyPressed = false;
    boolean sliderPosReseted = false;
    boolean clampServoOut = false;
    boolean tapeMoving = false;

    // DriveThru combos
    RobotControl currentTask = null;

    @Override
    public void init() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        fieldMode = true; //robot starts in field orientation

        Logger.init();

        robotHardware = RobotFactory.getRobotHardware(hardwareMap,robotProfile);
        robotHardware.setClampAnglePosition(RobotHardware.ClampAnglePosition.NORMAL);

        // Based on the Autonomous mode starting position, define the gyro offset for field mode
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);

        gyroAngleOffset = robotHardware.getGyroAngle();
        if (prefs.getString(START_POS_MODES_PREF, "").contains("RED")) {
            gyroAngleOffset += 90;
        }
        else {
            gyroAngleOffset -= 90;
        }

        navigator = new RobotNavigator(robotProfile);
        navigator.reset();
        navigator.setInitPosition(0, 0, 0);
//        if(!robotHardware.imu1.isGyroCalibrated()){
//
//        }
        setupCombos();

        driverOptions = new DriverOptions();

        try {
            driverOptions.setStartingPositionModes(prefs.getString(START_POS_MODES_PREF, ""));
            Logger.logFile("startingPositionModes: "+ driverOptions.getStartingPositionModes());
        } catch (Exception e) {
        }
    }

    @Override
    public void loop() {
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();

        gyroCurrAngle = robotHardware.getGyroAngle();

        // Robot movement
        handleMovement();

        // toggle field mode on/off.
        // Driver 1: left trigger - enable; right trigger - disable
        if (gamepad1.left_trigger > 0) {
            fieldMode = true;
            gyroAngleOffset = gyroCurrAngle;
        } else if (gamepad1.right_trigger > 0) {
            fieldMode = false;  //good luck driving
        }


//        if (gamepad1.x) {
//            robotHardware.setHookPosition(RobotHardware.HookPosition.HOOK_ON);
//        } else if (gamepad1.y) {
//            robotHardware.setHookPosition(RobotHardware.HookPosition.HOOK_OFF);
//        }

        if (!yAlreadyPressed && !gamepad2.start) {
            if (gamepad2.y) {
                //currentTask = homePositionTask;
                currentTask.prepare();
                Logger.logFile(currentTask.toString());
                yAlreadyPressed = true;
            }
        }

        if (!gamepad2.y) {
            yAlreadyPressed = false;
        }

        if (currentTask != null) {
            currentTask.execute();

            if (currentTask.isDone()) {
                currentTask.cleanUp();
                currentTask = null;
            }
        }

        telemetry.addData("Field Mode", fieldMode);
        telemetry.addData("Gyro", gyroCurrAngle);
    }

    @Override
    public void stop() {
        // open the clamp to relief the grabber servo
        try {
            Logger.flushToFile();
        } catch (Exception e) {
        }
    }

    private void handleMovement() {
        double turn = gamepad1.right_stick_x/2;
        double power = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double moveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI/4.5;

        if (fieldMode) {
            moveAngle += Math.toRadians(gyroCurrAngle - gyroAngleOffset);
        }

        if (gamepad1.left_bumper) {
            power = power/3.5;
            turn = turn/13;
        }
        if(gamepad1.x && !xAlreadyPressed){
            xAlreadyPressed = true;
            power = power/3.5;
            turn = turn/13;
        } else if(gamepad1.x && xAlreadyPressed){
            xAlreadyPressed = false;
            power = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            turn = gamepad1.right_stick_x;
        }

        robotHardware.mecanumDriveTest(power, moveAngle, turn, 0);
    }

    void setupCombos() {
//        ArrayList<RobotControl> homePositionList = new ArrayList<RobotControl>();
//        homePositionList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
//                robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra, 100));
//        homePositionList.add(new ClampStraightAngleTask(robotHardware, robotProfile));
//        homePositionList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOrigPos, 100));
//        homePositionList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
//        homePositionList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
//                robotProfile.hardwareSpec.liftHomeReadyPos, 100));
//        homePositionTask = new SequentialComboTask();
//        homePositionTask.setTaskList(homePositionList);
    }
}

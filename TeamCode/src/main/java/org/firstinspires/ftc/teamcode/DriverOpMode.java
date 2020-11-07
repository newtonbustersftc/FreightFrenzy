package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

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
    boolean yAlreadyPressed = false;

    // DriveThru combos
    RobotControl currentTask = null;
    RobotVision robotVision;
    @Override
    public void init() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        fieldMode = true; //robot starts in field orientation

        Logger.init();

        robotHardware = RobotFactory.getRobotHardware(hardwareMap,robotProfile);
        robotVision = robotHardware.getRobotVision();
        robotVision.activateRecognition();
        robotVision.activateNavigationTarget();

        // Based on the Autonomous mode starting position, define the gyro offset for field mode
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);

        gyroAngleOffset = robotHardware.getGyroAngle();

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

    private void handleVision() {
        Pose2d currentPosition = robotVision.getNavigationLocalization();
        telemetry.addData("Current Location", currentPosition);
        List<Recognition> updatedRecognitions = robotVision.getRingRecognition();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
            }
        }
    }

    @Override
    public void loop() {
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();

        gyroCurrAngle = robotHardware.getGyroAngle();

        // Robot movement
        handleMovement();
        handleVision();

        // toggle field mode on/off.
        // Driver 1: left trigger - enable; right trigger - disable
        if (gamepad1.left_trigger > 0) {
            fieldMode = true;
            gyroAngleOffset = gyroCurrAngle;
        } else if (gamepad1.right_trigger > 0) {
            fieldMode = false;  //good luck driving
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

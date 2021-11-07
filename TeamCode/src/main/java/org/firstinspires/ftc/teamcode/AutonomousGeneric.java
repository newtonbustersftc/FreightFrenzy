package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.util.ArrayList;

/**
 * 2019.10.26
 * Created by Ian Q.
 */
@Autonomous(name="Newton Autonomous", group="Main")
public class AutonomousGeneric extends LinearOpMode {

    RobotHardware robotHardware;
    RobotNavigator navigator;
    RobotProfile robotProfile;
    DriverOptions driverOptions;

    int leftEncoderCounts;
    int rightEncoderCounts;
    int horizontalEncoderCounts;

    ArrayList<RobotControl> taskList;

    long loopCount = 0;
    int countTasks = 0;
    private int delay;

    enum PfPose {
        RED_START, BLUE_START, RED_WAREHOUSE, BLUE_WAREHOUSE, RED_CAROUSEL, BLUE_CAROUSEL,
        RED_STORAGE, BLUE_STORAGE, RED_SHIPPINGHUB, BLUE_SHIPPINGHUB, RED_SHAREDHUB,
        BLUE_SHAREDHUB
    }

    public void initRobot() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        Logger.init();

        RobotFactory.reset();
        robotHardware = RobotFactory.getRobotHardware(hardwareMap,robotProfile);
        robotHardware.initRobotVision();
        robotHardware.setMotorStopBrake(true);

        robotHardware.getBulkData1();
        robotHardware.getBulkData2();

        Logger.logFile("Init completed");

        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        driverOptions.setStartingPositionModes(prefs.getString("START_POS_MODES_PREF", ""));
        driverOptions.setParking(prefs.getString("PARKING_PREF", ""));
        driverOptions.setDelay(prefs.getInt("DELAY_PREF", 0));
        driverOptions.setDeliveryRoutes(prefs.getString("DELIVERY_ROUTES_PREF", ""));
    }

    @Override
    public void runOpMode() {
        initRobot();
        robotHardware.setMotorStopBrake(false); // so we can adjust the robot

        // reset arm position
        int warningSoundID = hardwareMap.appContext.getResources().getIdentifier("backing_up", "raw", hardwareMap.appContext.getPackageName());
        if (warningSoundID != 0) {
            Logger.logFile("Found warning sound backing_up");
            if (SoundPlayer.getInstance().preload(hardwareMap.appContext, warningSoundID)) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, warningSoundID);
            }
        }
        telemetry.addData("READY...", "NOW");
        waitForStart();
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();

        robotHardware.getLocalizer().update();
        robotHardware.getMecanumDrive().setPoseEstimate(getProfilePose("START_STATE"));
        robotHardware.setMotorStopBrake(true);  // so no sliding when we move
        RobotVision.AutonomousGoal goal = robotHardware.getRobotVision().getAutonomousRecognition();
        Logger.logFile("recognition result: " + goal);
        AutonomousTaskBuilder builder = new AutonomousTaskBuilder(driverOptions, robotHardware, robotProfile, goal);
        //goWobbleTask(driverOptions.getStartingPositionModes());
        //if (driverOptions.getDuckPosition().contains("yes")) {//5 points - do nothing but parking
        //    taskList = builder.buildParkingOnlyTask(driverOptions.getParking());
        //}
        // do the task list building after click start, which we should have the skystone position
        //AutonomousTaskBuilder builder = new AutonomousTaskBuilder(driverOptions, skystonePosition, robotHardware, navigator, robotProfile);
        taskList = new ArrayList<RobotControl>();
        //prepareTaskList(goal);



        TaskReporter.report(taskList);
        Logger.logFile("Task list items: " + taskList.size());
        Logger.flushToFile();

        if (taskList.size() > 0) {
            taskList.get(0).prepare();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            loopCount++;

            robotHardware.getBulkData1();
            robotHardware.getBulkData2();

            if (taskList.size() > 0) {
                taskList.get(0).execute();

                if (taskList.get(0).isDone()) {
                    Logger.logFile("MainTaskComplete: " + taskList.get(0) + " Pose:" + robotHardware.getLocalizer().getPoseEstimate());
                    Logger.flushToFile();

                    taskList.get(0).cleanUp();
                    taskList.remove(0);

                    countTasks++;
                    telemetry.update();

                    if (taskList.size() > 0) {
                        taskList.get(0).prepare();
                    }
                }
            }
        }
        // Regardless, open the clamp to save the servo
        try {
            Logger.logFile("Autonomous - Final Location:" + navigator.getLocationString());
            Logger.flushToFile();
        } catch (Exception ex) {
        }

        robotHardware.setMotorStopBrake(false);
    }

    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }








}

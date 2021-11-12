package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.util.ArrayList;

@Autonomous(name="Newton Autonomous", group="Main")
public class AutonomousGeneric extends LinearOpMode {

    RobotHardware robotHardware;
    RobotNavigator navigator;
    RobotProfile robotProfile;
    DriverOptions driverOptions;
//    RobotVision robotVision;

    ArrayList<RobotControl> taskList;

    long loopCount = 0;
    int countTasks = 0;
    private int delay;

    public void initRobot() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        }
        catch (Exception e) {
            RobotLog.e("RobotProfile reading exception" + e);
        }

        Logger.init();

        RobotFactory.reset();

//        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);

//        robotHardware.setMotorStopBrake(true);

        resetLiftPosition();

        robotHardware.getBulkData1();
        robotHardware.getBulkData2();

        driverOptions = new DriverOptions();
        Logger.logFile("Init completed");
        try {
            SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
            String delayString = prefs.getString("DELAY_PREF", "").replace(" sec", "");
            driverOptions.setDelay(Integer.parseInt(delayString));
            Logger.logFile("delay: " + driverOptions.getDelay());

            driverOptions.setStartingPositionModes(prefs.getString("START_POS_MODES_PREF", ""));
//            driverOptions.setParking(prefs.getString("PARKING_PREF", ""));
//            driverOptions.setDeliveryRoutes(prefs.getString("DELIVERY_ROUTES_PREF", ""));
//            driverOptions.setDoCarousel(prefs.getString("DUCK_CAROUSEL", "").equals("yes") ? true : false);
            Logger.logFile("starting position: " + driverOptions.getStartingPositionModes());
            Logger.logFile("parking: " + driverOptions.getParking());
            Logger.logFile("delivery route: " + driverOptions.getDeliveryRoutes());
        }
        catch (Exception e) {
            this.delay = 0;
        }
    }

    @Override
    public void runOpMode() {
        initRobot();
        robotHardware.setMotorStopBrake(false); // so we can adjust the robot
        robotHardware.initRobotVision();
        resetLiftPosition();
        telemetry.addData("READY...", "NOW");
        waitForStart();
//        robotHardware.getBulkData1();
//        robotHardware.getBulkData2();

//        robotHardware.getLocalizer().update();
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0, 0, 0));
//        robotHardware.getMecanumDrive().setPoseEstimate(getProfilePose("START_STATE"));

//        robotHardware.setMotorStopBrake(true);  // so no sliding when we move

        RobotVision.AutonomousGoal goal = robotHardware.getRobotVision().getAutonomousRecognition();
        Logger.logFile("recognition result: " + goal);

        AutonomousTaskBuilder builder = new AutonomousTaskBuilder(driverOptions, robotHardware, robotProfile, goal);

        taskList = builder.buildTaskList(goal);

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
            robotHardware.getLocalizer().update();

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
        }
        catch (Exception ex) {
        }

        robotHardware.setMotorStopBrake(false);
    }

    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }

    void resetLiftPosition() {
        Logger.logFile("Resetting Lift Position, bottom sensor: " + robotHardware.liftBottomTouched());
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();
        while (!robotHardware.liftBottomTouched()) {
            int currLiftPos = robotHardware.getEncoderCounts(RobotHardware.EncoderType.LIFT);
            Logger.logFile("Lift current Position " + currLiftPos);
            robotHardware.setLiftMotorPosition(currLiftPos-50);
            try {
                Thread.sleep(100);
            }
            catch (Exception e) {
            }
        }
        robotHardware.resetLiftEncoderCount();
        robotHardware.setLiftPosition(RobotHardware.LiftPosition.ZERO);
    }
}

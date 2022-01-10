package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;

@Autonomous(name="Newton Autonomous", group="Main")
public class AutonomousGeneric extends LinearOpMode {

    RobotHardware robotHardware;
    RobotProfile robotProfile;
    DriverOptions driverOptions;
//    RobotVision robotVision;

    ArrayList<RobotControl> taskList;

    long loopCount = 0;
    int countTasks = 0;
    private int delay;

    public void initRobot() {
        try {
            robotProfile = RobotProfile.loadFromFile();
        }
        catch (Exception e) {
            RobotLog.e("RobotProfile reading exception" + e);
        }

        Logger.init();

        RobotFactory.reset();

        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
//        robotHardware = new RobotHardware();
//        robotHardware.init(hardwareMap, robotProfile);

//        robotHardware.setMotorStopBrake(true);

        robotHardware.resetLiftPositionAutonomous();
        robotHardware.closeBoxFlap();

        robotHardware.getBulkData1();
        robotHardware.getBulkData2();

        driverOptions = new DriverOptions();
        Logger.logFile("Init completed");
        try {
            SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
            String delayString = prefs.getString("delay", "0").replace(" sec", "");
            driverOptions.setDelay(Integer.parseInt(delayString));
            Logger.logFile("delay: " + driverOptions.getDelay());
            String freightDeliveryCount = prefs.getString("freightDeliveryCount", "0").replace(" sec", "");
            driverOptions.setFreightDeliveryCount(Integer.parseInt(freightDeliveryCount));
            Logger.logFile("delay: " + driverOptions.getFreightDeliveryCount());


            driverOptions.setStartingPositionModes(prefs.getString("starting position", ""));
            driverOptions.setParking(prefs.getString("parking", ""));
            driverOptions.setDeliveryRoutes(prefs.getString("delivery routes", ""));
            String delay_parking = prefs.getString("delay parking", "0").replace( " sec", "");
            driverOptions.setDelayParking(Integer.parseInt(delay_parking));
            driverOptions.setParkingOnly(prefs.getString("park only", ""));
            Logger.logFile("starting position: " + driverOptions.getStartingPositionModes());
            Logger.logFile("parking: " + driverOptions.getParking());
            Logger.logFile("parking_delay: " + driverOptions.getDelayParking());
            Logger.logFile("delivery route: " + driverOptions.getDeliveryRoutes());
            Logger.logFile("park only: " + driverOptions.getParkingOnly());
        }
        catch (Exception e) {
            RobotLog.e("SharedPref exception " + e);
            this.delay = 0;
        }
        Logger.logFile("Done with init in autonomous");
    }

    @Override
    public void runOpMode() {
        initRobot();
        robotHardware.setMotorStopBrake(false); // so we can adjust the robot00
        robotHardware.initRobotVision();
        robotHardware.getRobotVision().initRearCamera(driverOptions.getStartingPositionModes().contains("RED"));  //boolean isRed

        robotHardware.resetLiftPositionAutonomous();
        long loopStart = System.currentTimeMillis();
        long loopCnt = 0;

        //warmUpT265();

        while (!isStarted()) {
            robotHardware.getLocalizer().update();
            Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
            loopCnt++;
            if (loopCnt==10000) {
                // Set to 0,0,0 for once
                robotHardware.getLocalizer().setPoseEstimate(new Pose2d());
            }
            if (loopCnt%10000==0) {
                RobotVision.AutonomousGoal goal = robotHardware.getRobotVision().getAutonomousRecognition(false);
                telemetry.addData("goal",goal);
//                telemetry.addData("CurrPose", currPose);
                telemetry.addData("T265 CFD:",  ((RealSenseLocalizer)robotHardware.getLocalizer()).getT265Confidence());
                telemetry.addData("LoopTPS:", (loopCnt * 1000 / (System.currentTimeMillis() - loopStart)));
                telemetry.addData("Profile:", robotProfile.fileDateStr);
                telemetry.addData("Starting Position:", driverOptions.getStartingPositionModes());
                telemetry.addData("Parking:", driverOptions.getParking());
                telemetry.update();
            }
        }
//        robotHardware.getBulkData1();
//        robotHardware.getBulkData2();

//        robotHardware.getLocalizer().update();
        robotHardware.resetImu();
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
        robotHardware.setMotorStopBrake(true);
        robotHardware.robotVision.startRearCamera();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && taskList.size()>0) {
            loopCount++;

            robotHardware.getBulkData1();
            robotHardware.getBulkData2();
            robotHardware.getLocalizer().update();

            if (!((RealSenseLocalizer)robotHardware.getLocalizer()).getT265Confidence().equals("High")){
                taskList.clear();
            }
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
            Logger.logFile("Autonomous - Final Location:" + robotHardware.getLocalizer().getPoseEstimate());
            Logger.flushToFile();
        }
        catch (Exception ex) {
        }

        robotHardware.getRobotVision().stopRearCamera();
        robotHardware.stopAll();
        robotHardware.setMotorStopBrake(false);
    }

    void warmUpT265() {
        boolean warmUp = false;
        long loopStart = System.currentTimeMillis();
        long loopCnt = 0;
        while (!warmUp && !isStarted()) {
            robotHardware.getLocalizer().update();
            Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
            loopCnt++;
            if (loopCnt==1000) {
                // Set to 0,0,0 for once
                robotHardware.getLocalizer().setPoseEstimate(new Pose2d());
            }
            if (loopCnt%1000==0) {
                telemetry.addData("Profile:", robotProfile.fileDateStr);
                telemetry.addLine("A to warm up, B to skip");
                telemetry.addData("CurrPose", currPose);
                telemetry.addData("T265 CFD:",  ((RealSenseLocalizer)robotHardware.getLocalizer()).getT265Confidence());
                telemetry.addData("LoopTPS:", (loopCnt * 1000 / (System.currentTimeMillis() - loopStart)));
                telemetry.update();
            }
            warmUp = gamepad1.a || gamepad1.b;
        }
        if (gamepad1.a) {
            int warningSoundID = hardwareMap.appContext.getResources().getIdentifier("backing_up", "raw", hardwareMap.appContext.getPackageName());
            if (warningSoundID != 0) {
                Logger.logFile("Found warning sound backing_up");
                if (SoundPlayer.getInstance().preload(hardwareMap.appContext, warningSoundID)) {
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, warningSoundID);
                }
            }

            robotHardware.getLocalizer().setPoseEstimate(new Pose2d());
            SampleMecanumDrive drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();
            TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .waitSeconds(3)
                    .back(20)
                    .waitSeconds(0.5)
                    .forward(20)
                    .waitSeconds(0.5)
                    .back(20)
                    .waitSeconds(0.5)
                    .forward(20)
                    .waitSeconds(0.5)
                    .build();
            drive.followTrajectorySequence(trajectory);
        }
    }

    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }

}

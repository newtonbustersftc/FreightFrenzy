package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
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

    int leftEncoderCounts;
    int rightEncoderCounts;
    int horizontalEncoderCounts;

    ArrayList<RobotControl> taskList;

    long loopCount = 0;
    int countTasks = 0;
    private int delay;

    enum PfPose {
        START, TRANSIT, SHOOT, ZONE_A1, ZONE_A1B, ZONE_B1, ZONE_B1B,
        ZONE_C1, ZONE_C1B, WOBBLE2_RDY, WOBBLE2_GRB
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
        robotHardware.setShooterPosition(false);

        robotHardware.getBulkData1();
        robotHardware.getBulkData2();

        Logger.logFile("Init completed");

        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
    }

    @Override
    public void runOpMode() {
        initRobot();
        robotHardware.setMotorStopBrake(false); // so we can adjust the robot
        robotHardware.ringHolderDown();
        robotHardware.setShooterPosition(false);
        robotHardware.setGrabberPosition(false);

        // reset arm position
        int warningSoundID = hardwareMap.appContext.getResources().getIdentifier("backing_up", "raw", hardwareMap.appContext.getPackageName());
        if (warningSoundID != 0) {
            Logger.logFile("Found warning sound backing_up");
            if (SoundPlayer.getInstance().preload(hardwareMap.appContext, warningSoundID)) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, warningSoundID);
            }
        }
        int lastArmPos = 0;

        boolean firstTime = true;
        while (true) {
            robotHardware.getBulkData1();
            robotHardware.getBulkData2();
            int currArmPos = robotHardware.getEncoderCounts(RobotHardware.EncoderType.ARM);
            if (!firstTime) {
                if (currArmPos == lastArmPos) {
                    // stop moving, we move up a bit then reset Arm motor to 0 position
                    robotHardware.setDirectArmMotorPos(lastArmPos+robotProfile.hardwareSpec.armReverseDelta);
                    try {
                        Thread.sleep(robotProfile.hardwareSpec.armReverseDelay);
                    }
                    catch (Exception e) {
                    }
                    robotHardware.resetArmMotorPosition();
                    break;  // done
                }
            }

            lastArmPos = currArmPos;
            robotHardware.setDirectArmMotorPos(lastArmPos - robotProfile.hardwareSpec.armReverseDelta);
            firstTime = false;
            try {
                Thread.sleep(robotProfile.hardwareSpec.armReverseDelay);
            }
            catch (Exception e) {
            }
        }
        if (warningSoundID!=0) {
            SoundPlayer.getInstance().stopPlayingAll();
        }
        robotHardware.setArmMotorPos(RobotHardware.ArmPosition.INIT);
        robotHardware.setGrabberPosition(true);
        try {
            Thread.sleep(3000); // put the wobble goal on
        }
        catch (Exception e) {
        }
        robotHardware.setGrabberPosition(false);
        telemetry.addData("READY...", "NOW");
        waitForStart();
        robotHardware.startShootMotor();
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();

        robotHardware.getTrackingWheelLocalizer().update();
        robotHardware.getMecanumDrive().setPoseEstimate(getProfilePose("START_STATE"));
        robotHardware.setMotorStopBrake(true);  // so no sliding when we move
        RobotVision.AutonomousGoal goal = robotHardware.getRobotVision().getAutonomousRecognition();
        Logger.logFile("recognition result: " + goal);

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
                    Logger.logFile("MainTaskComplete: " + taskList.get(0) + " Pose:" + robotHardware.getTrackingWheelLocalizer().getPoseEstimate());
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

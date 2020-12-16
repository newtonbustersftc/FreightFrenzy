package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.AutonomousOptions.DELAY_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.PARKING_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.FOUNDATION_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.DELIVER_ROUTE_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.PARKING_ONLY_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.STONE_PREF;

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
        robotHardware.setMotorStopBrake(true);

        robotHardware.getBulkData1();
        robotHardware.getBulkData2();

        Logger.logFile("Init completed");

        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
    }

    @Override
    public void runOpMode() {
        initRobot();
        robotHardware.setMotorStopBrake(false); // so we can adjust the robot
        robotHardware.setGrabberPosition(true);
        try {
            Thread.sleep(3000); // put the wobble goal on
        }
        catch (Exception e) {
        }
        robotHardware.setGrabberPosition(false);
        telemetry.addData("READY...", "NOW");
        waitForStart();
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();

        robotHardware.getTrackingWheelLocalizer().update();
        robotHardware.getMecanumDrive().setPoseEstimate(getProfilePose("START"));
        robotHardware.setMotorStopBrake(true);  // so no sliding when we move
        RobotVision.AutonomousGoal goal = robotHardware.getRobotVision().getAutonomousRecognition();
        Logger.logFile("recognition result: " + goal);

        // do the task list building after click start, which we should have the skystone position
        //AutonomousTaskBuilder builder = new AutonomousTaskBuilder(driverOptions, skystonePosition, robotHardware, navigator, robotProfile);
        taskList = new ArrayList<RobotControl>();
        //prepareTaskList(goal);
        prepareNoneTaskList();

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

    Pose2d getProfilePose(PfPose p) {
        return new Pose2d();
    }

    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }

    void prepareTaskList(RobotVision.AutonomousGoal goal) {
        // Move 1 - to shooting position
        Trajectory trjToShoot = robotHardware.mecanumDrive.trajectoryBuilder(getProfilePose(PfPose.START),
                                    new DriveConstraints(20, 10, 0.0,
                                    Math.toRadians(180.0), Math.toRadians(180.0), 0.0))
                    .splineTo(getProfilePose(PfPose.TRANSIT).vec(), getProfilePose(PfPose.TRANSIT).getHeading() )
                    .splineTo(getProfilePose(PfPose.SHOOT).vec(), getProfilePose(PfPose.SHOOT).getHeading())
                    .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trjToShoot);
        ParallelComboTask par1 = new ParallelComboTask();
        par1.addTask(moveTask1);
        par1.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 1000));
        taskList.add(par1);
        // Shooting action
        taskList.add(new RobotSleep(2000)); // pretend we are shooting

        // Move 2 - to Wobble drop position
        PfPose wbPose1, wbPose2;
        if (goal==RobotVision.AutonomousGoal.NONE) {
            wbPose1 = PfPose.ZONE_A1;
            wbPose2 = PfPose.ZONE_A1B;
        }
        else if (goal==RobotVision.AutonomousGoal.SINGLE) {
            wbPose1 = PfPose.ZONE_B1;
            wbPose2 = PfPose.ZONE_B1B;
        }
        else {
            wbPose1 = PfPose.ZONE_C1;
            wbPose2 = PfPose.ZONE_C1B;
        }
        Trajectory trjToDrop = robotHardware.mecanumDrive.trajectoryBuilder(getProfilePose(PfPose.SHOOT),
                new DriveConstraints(20, 10, 0.0,
                        Math.toRadians(180.0), Math.toRadians(180.0), 0.0))
                .splineToSplineHeading(getProfilePose(wbPose1), getProfilePose(wbPose1).getHeading())
                .build();
        SplineMoveTask moveTask2 = new SplineMoveTask(robotHardware.mecanumDrive, trjToDrop);
        taskList.add(moveTask2);
        // Wobble goal dropping
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 500)); // pretend we are dropping
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 500));
        // Move 3 - move back
        Trajectory trjBack1 = robotHardware.mecanumDrive.trajectoryBuilder(getProfilePose(wbPose1),
                new DriveConstraints(20, 10, 0.0,
                        Math.toRadians(180.0), Math.toRadians(180.0), 0.0))
                .lineTo(getProfilePose(wbPose2).vec())
                .build();
        SplineMoveTask moveTask3 = new SplineMoveTask(robotHardware.mecanumDrive, trjBack1);
        taskList.add(moveTask3);
        taskList.add(new GrabberTask(robotHardware, robotProfile, false, 500));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 500)); // pretend we are dropping
        // Move 4 - move to wobble 2
        Trajectory trjToW2 = robotHardware.mecanumDrive.trajectoryBuilder(getProfilePose(wbPose2),
                new DriveConstraints(20, 10, 0.0,
                        Math.toRadians(180.0), Math.toRadians(180.0), 0.0))
                .splineToSplineHeading(getProfilePose(PfPose.WOBBLE2_RDY), getProfilePose(PfPose.WOBBLE2_RDY).getHeading())
                .splineTo(getProfilePose(PfPose.WOBBLE2_GRB).vec(), getProfilePose(PfPose.WOBBLE2_GRB).getHeading())
                .build();
        SplineMoveTask moveTask4 = new SplineMoveTask(robotHardware.mecanumDrive, trjToW2);
        taskList.add(moveTask4);
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 500));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 500));
    }

    void prepareNoneTaskList() {
        // Move 1 - to shooting position
        DriveConstraints constraints = new DriveConstraints(20.0, 10.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);

        Pose2d p0 = getProfilePose("START");
        Pose2d p1 = getProfilePose("SHOOT");
        Trajectory trjShoot = robotHardware.mecanumDrive.trajectoryBuilder(p0, constraints)
                .splineTo(p1.vec(), p1.getHeading())
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trjShoot);
        ParallelComboTask par1 = new ParallelComboTask();
        par1.addTask(moveTask1);
        par1.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 1000));
        taskList.add(par1);

        // Shooting action
        taskList.add(new RobotSleep(2000)); // pretend we are shooting

        Pose2d p2 = getProfilePose("A-1");
        Trajectory trjWob = robotHardware.mecanumDrive.trajectoryBuilder(p1, constraints)
                .splineToSplineHeading(p2, p2.getHeading())
                .build();
        SplineMoveTask moveTask2 = new SplineMoveTask(robotHardware.mecanumDrive, trjWob);
        taskList.add(moveTask2);
        // Wobble goal dropping
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 500));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 500));
        // Move to Wobble 2
        Pose2d p3pre = getProfilePose("WB-2Pre");
        Pose2d p3 = getProfilePose("WB-2");
        Trajectory trjPickup = robotHardware.mecanumDrive.trajectoryBuilder(p2, true)
                .splineToSplineHeading(p3pre, p3pre.getHeading(), constraints)
                .splineToSplineHeading(p3, p3.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask3 = new SplineMoveTask(robotHardware.mecanumDrive, trjPickup);
        taskList.add(moveTask3);
        // Grab wobble 2
        taskList.add(new GrabberTask(robotHardware, robotProfile, false, 500));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 500));

        Pose2d p4 = getProfilePose("A-2");
        Trajectory trjWob2 = robotHardware.mecanumDrive.trajectoryBuilder(p3, true)
                .splineToSplineHeading(p4, p4.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask4 = new SplineMoveTask(robotHardware.mecanumDrive, trjWob2);
        taskList.add(moveTask4);
        // Drop wobble 2
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 500));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 500));

    }
}

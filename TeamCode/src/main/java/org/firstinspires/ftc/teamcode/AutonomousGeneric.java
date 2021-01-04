package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
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

//        if (RobotVision.AutonomousGoal.SINGLE==goal) {
//            prepareSingleTaskList();
//        }
//        else if (RobotVision.AutonomousGoal.QUAD==goal) {
//            prepareQuadTaskList();
//        }
//        else {
//            prepareNoneTaskList();
//        }

        prepareBarTaskList();

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

    void prepareQuadTaskList() {
        DriveConstraints constraints = new DriveConstraints(20.0, 10.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);

        // move to shoot
        Pose2d p0 = getProfilePose("START");
        Pose2d p1 = getProfilePose("TRANSIT");
        Pose2d p2 = getProfilePose("SHOOT");
        Trajectory trjShoot = robotHardware.mecanumDrive.trajectoryBuilder(p0, moveFast)
                .splineTo(p1.vec(), p1.getHeading())
                .splineToSplineHeading(p2, p2.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trjShoot);
        ParallelComboTask par1 = new ParallelComboTask();
        par1.addTask(moveTask1);
        par1.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 1000));
        par1.addTask(new ShooterMotorTask(robotHardware, robotProfile, true));
        taskList.add(par1);
        // Shooting action
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, false));

        Pose2d p3 = getProfilePose("C-1");
        Trajectory trjWob = robotHardware.mecanumDrive.trajectoryBuilder(p2, moveFast)
                .splineToSplineHeading(p3, p3.getHeading())
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjWob));
        // Wobble goal dropping
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 500));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 500));
        // Move to grab wobble 2
        Pose2d p4 = getProfilePose("TRANSIT2");
        Pose2d p5 = getProfilePose("C-WB2Pre");
        Pose2d p6 = getProfilePose("C-WB2");
        Trajectory pickUp = robotHardware.mecanumDrive.trajectoryBuilder(p3, true)
                .splineToSplineHeading(p4, p4.getHeading(), moveFast)
                .splineTo(p5.vec(), p5.getHeading(), moveFast)
                .splineTo(p6.vec(), p6.getHeading(), constraints)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, pickUp));
        // Grab wobble 2
        taskList.add(new GrabberTask(robotHardware, robotProfile, false, 500));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 500));
        // Move to drop wobble 2
        Pose2d p7pre = getProfilePose("TRANSIT3");
        Pose2d p7 = getProfilePose("C-2");
        Trajectory trjWob2 = robotHardware.mecanumDrive.trajectoryBuilder(p6, moveFast)
                .splineTo(p7pre.vec(), p7pre.getHeading())
                .splineTo(p7.vec(), p7.getHeading(), moveFast)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjWob2));
        // Drop wobble 2
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 500));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 500));

        // move to parking
        Trajectory trjPark = robotHardware.mecanumDrive.trajectoryBuilder(p7, moveFast)
                .back(30)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjPark));
    }

    void prepareBarTaskList() {
        DriveConstraints constraints = new DriveConstraints(20.0, 10.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);

        // move to shoot
        Pose2d p0 = getProfilePose("START");
        Pose2d p1 = getProfilePose("SHOOT-POWER-BAR-1");
        Pose2d p2 = getProfilePose("SHOOT-POWER-BAR-2");
        Pose2d p3 = getProfilePose("SHOOT-POWER-BAR-3");
        Trajectory trjShoot = robotHardware.mecanumDrive.trajectoryBuilder(p0, moveFast)
                .splineTo(p1.vec(), p1.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trjShoot);
        ParallelComboTask par1 = new ParallelComboTask();
        par1.addTask(moveTask1);
        par1.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 1000));
        par1.addTask(new ShooterMotorTask(robotHardware, robotProfile, true, robotProfile.hardwareSpec.shootPowerBar));
        taskList.add(par1);
        // Shooting action
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));

        ParallelComboTask par2 = new ParallelComboTask();
        MecanumRotateTask rotateTask1 = new MecanumRotateTask(robotHardware.mecanumDrive, p2.getHeading() - p1.getHeading());
        par2.addTask(rotateTask1);
        par2.addTask(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(par2);
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));

        ParallelComboTask par3 = new ParallelComboTask();
        MecanumRotateTask rotateTask2 = new MecanumRotateTask(robotHardware.mecanumDrive, p3.getHeading() - p2.getHeading());
        par3.addTask(rotateTask2);
        par3.addTask(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(par3);
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));

        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));

        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, false));
    }

    void prepareSingleTaskList() {
        DriveConstraints constraints = new DriveConstraints(20.0, 10.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        // move to shoot
        Pose2d p0 = getProfilePose("START");
        Pose2d p1 = getProfilePose("TRANSIT");
        Pose2d p2 = getProfilePose("SHOOT");
        Trajectory trjShoot = robotHardware.mecanumDrive.trajectoryBuilder(p0, moveFast)
                .splineTo(p1.vec(), p1.getHeading())
                .splineToSplineHeading(p2, p2.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trjShoot);
        ParallelComboTask par1 = new ParallelComboTask();
        par1.addTask(moveTask1);
        par1.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 1000));
        par1.addTask(new ShooterMotorTask(robotHardware, robotProfile, true));
        taskList.add(par1);
        // Shooting action
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, false));
        // move to drop wobble 1
        Pose2d p3 = getProfilePose("B-1");;
        Trajectory trjWob = robotHardware.mecanumDrive.trajectoryBuilder(p2, moveFast)
                .splineToSplineHeading(p3, p3.getHeading())
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjWob));
        // Wobble goal dropping
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 500));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 500));
        // Move to grab wobble 2
        Pose2d p4 = getProfilePose("TRANSIT2");
        Pose2d p5 = getProfilePose("B-WB2Pre");
        Pose2d p6 = getProfilePose("B-WB2");
        Trajectory pickUp = robotHardware.mecanumDrive.trajectoryBuilder(p3, true)
                .splineToSplineHeading(p4, p4.getHeading(), moveFast)
                .splineTo(p5.vec(), p5.getHeading(), moveFast)
                .splineTo(p6.vec(), p6.getHeading(), constraints)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, pickUp));
        // Grab wobble 2
        taskList.add(new GrabberTask(robotHardware, robotProfile, false, 500));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 500));
        // Move to drop wobble 2
        Pose2d p7pre = getProfilePose("TRANSIT3");
        Pose2d p7 = getProfilePose("B-2");
        Trajectory trjWob2 = robotHardware.mecanumDrive.trajectoryBuilder(p6, moveFast)
                .splineTo(p7pre.vec(), p7pre.getHeading())
                .splineTo(p7.vec(), p7.getHeading())
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjWob2));
        // Drop wobble 2
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 500));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 500));

        // move to parking
        Trajectory trjPark = robotHardware.mecanumDrive.trajectoryBuilder(p7, moveFast)
                .back(10)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjPark));
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
        par1.addTask(new ShooterMotorTask(robotHardware, robotProfile, true));
        taskList.add(par1);

        // Shooting action
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, false));

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
        Pose2d p3pre = getProfilePose("A-WB2Pre");
        Pose2d p3 = getProfilePose("A-WB2");
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

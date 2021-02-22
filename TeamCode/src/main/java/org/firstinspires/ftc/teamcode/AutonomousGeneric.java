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

        if (RobotVision.AutonomousGoal.SINGLE==goal) {
            prepareSingleTaskList();
        }
        else if (RobotVision.AutonomousGoal.QUAD==goal) {
            prepareQuadTaskList();
        }
        else {
            prepareNoneTaskList();
        }

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
        par1.addTask(new ShooterMotorTask(robotHardware, robotProfile, true, robotProfile.hardwareSpec.shootBarVelocity));
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

    void prepareQuadTaskList() {
        DriveConstraints constraints = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints extraSlowConstraints = new DriveConstraints(5.0, 5.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints moveFast = new DriveConstraints(40.0, 30.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);

        // move to shoot 1
        Pose2d p0 = getProfilePose("START");
        Pose2d p1 = getProfilePose("TRANSIT");
        Pose2d p2 = getProfilePose("SHOOT");
        Trajectory trjShoot = robotHardware.mecanumDrive.trajectoryBuilder(p0, constraints)
                .splineTo(p1.vec(), p1.getHeading())
                .splineToSplineHeading(p2, p2.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trjShoot);
        ParallelComboTask par1 = new ParallelComboTask();
        par1.addTask(moveTask1);
        par1.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        par1.addTask(new ShooterMotorTask(robotHardware, robotProfile, true));
        taskList.add(par1);
        // Shooting action
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        //taskList.add(new ShooterMotorTask(robotHardware, robotProfile, false));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 10));

        Pose2d p3 = getProfilePose("C-1");
        Trajectory trjWob = robotHardware.mecanumDrive.trajectoryBuilder(p2, moveFast)
                .splineToSplineHeading(p3, p3.getHeading(), moveFast)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjWob));
        // Wobble goal dropping
        //taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 100));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 100));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 10));
        taskList.add(new IntakeMotorTask(robotHardware, robotProfile, IntakeMotorTask.IntakeMode.NORMAL));

        //move to pick up two rings for shoot 2
        ParallelComboTask par2 = new ParallelComboTask();
        par2.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));

        Pose2d p4 = getProfilePose("C-PICK");
        Pose2d p5 = getProfilePose("C-PICK2");
        Pose2d p6 = getProfilePose("C-PICK3Back");
        Trajectory pickUp = robotHardware.mecanumDrive.trajectoryBuilder(p3, true)
                .splineToSplineHeading(p4, p4.getHeading(), moveFast)
                .splineToSplineHeading(p5, p5.getHeading(), moveFast)
                .splineToSplineHeading(p6, p6.getHeading(), moveFast)
                .build();
        par2.addTask(new SplineMoveTask(robotHardware.mecanumDrive, pickUp));
        taskList.add(par2);

        //move to shoot 2
        ParallelComboTask shoot_2 = new ParallelComboTask();
        shoot_2.addTask(new ShooterMotorTask(robotHardware, robotProfile, true));

        Pose2d p7 = getProfilePose("SHOOT-DRIVER");
        Trajectory trjShoot2 = robotHardware.mecanumDrive.trajectoryBuilder(p6, true)
                .splineToSplineHeading(p7, p7.getHeading(), moveFast)
                .build();
        shoot_2.addTask(new SplineMoveTask(robotHardware.mecanumDrive, trjShoot2));
        taskList.add(shoot_2);
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));

        //move to shoot 3
        ParallelComboTask par3 = new ParallelComboTask();
        // Rotate 180  degree
        par3.addTask(new MecanumRotateTask(robotHardware.getMecanumDrive(), -Math.PI));
        par3.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));
        taskList.add(par3);

        taskList.add(new AutoDriveShootTask(robotHardware, robotProfile, AutoDriveShootTask.TaskMode.FIRST_PIC));
        taskList.add(new AutoDriveShootTask(robotHardware, robotProfile, AutoDriveShootTask.TaskMode.DRIVE));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));  // extra time for ring to bucket
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        // shoot last ring and move to parking
        ParallelComboTask par4 = new ParallelComboTask();
        par4.addTask(new ShootOneRingTask(robotHardware, robotProfile));
        Pose2d shoot2 = getProfilePose("SHOOT-DRIVER");
        Trajectory trjPark = robotHardware.mecanumDrive.trajectoryBuilder(shoot2, moveFast)
                .forward(5)
                .build();
        par4.addTask(new SplineMoveTask(robotHardware.mecanumDrive, trjPark));
        par4.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));
        par4.addTask(new ShooterMotorTask(robotHardware, robotProfile, false));
        taskList.add(par4);
        taskList.add(new IntakeMotorTask(robotHardware, robotProfile, IntakeMotorTask.IntakeMode.STOP));
    }

    void prepareSingleTaskList() {
        DriveConstraints constraints = new DriveConstraints(20.0, 10.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        // move to shoot
        Pose2d p0 = getProfilePose("START");
        Pose2d p1 = getProfilePose("TRANSIT");
        Pose2d p2 = getProfilePose("SHOOT");
        Trajectory trjShoot = robotHardware.mecanumDrive.trajectoryBuilder(p0, constraints)
                .splineTo(p1.vec(), p1.getHeading())
                .splineToSplineHeading(p2, p2.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trjShoot);
        ParallelComboTask par1 = new ParallelComboTask();
        par1.addTask(moveTask1);
        par1.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        par1.addTask(new ShooterMotorTask(robotHardware, robotProfile, true));
        taskList.add(par1);
        // Shooting action
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        //taskList.add(new ShooterMotorTask(robotHardware, robotProfile, false));
        // move to drop wobble 1
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 10));
        Pose2d p3 = getProfilePose("B-1");;
        Trajectory trjWob = robotHardware.mecanumDrive.trajectoryBuilder(p2, moveFast)
                .splineToSplineHeading(p3, p3.getHeading())
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjWob));
        // Wobble goal dropping
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 1000));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 200));
        // pick up ring
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));
        taskList.add(new IntakeMotorTask(robotHardware, robotProfile, IntakeMotorTask.IntakeMode.NORMAL));
        // Move to grab wobble 2
        Pose2d p4 = getProfilePose("B-PICK");
        Pose2d p5 = getProfilePose("B-WB2Pre");
        Pose2d p6 = getProfilePose("B-WB2");
        Trajectory pickUp = robotHardware.mecanumDrive.trajectoryBuilder(p3, true)
                .splineToSplineHeading(p4, p4.getHeading())
                .splineTo(p5.vec(), p5.getHeading())
                .splineTo(p6.vec(), p6.getHeading())
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, pickUp));

        // Grab wobble 2
        taskList.add(new GrabberTask(robotHardware, robotProfile, false, 500));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 10));
        // Move to shoot
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        taskList.add(new IntakeMotorTask(robotHardware, robotProfile, IntakeMotorTask.IntakeMode.STOP));
        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, true));
        Pose2d p7 = getProfilePose("SHOOT-DRIVER");
        Trajectory trjShoot2 = robotHardware.mecanumDrive.trajectoryBuilder(p6, true)
                .splineToSplineHeading(p7, p7.getHeading())
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjShoot2));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
//        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, false));

        // Move to drop wobble 2
        Pose2d p8 = getProfilePose("B-2");
        Trajectory trjWob2 = robotHardware.mecanumDrive.trajectoryBuilder(p7, moveFast)
                .splineTo(p8.vec(), p8.getHeading())
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjWob2));
        // Drop wobble 2
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 1500));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 500));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 10));

        // move to parking
        Trajectory trjPark = robotHardware.mecanumDrive.trajectoryBuilder(p8, moveFast)
                .back(10)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjPark));
    }

    void prepareNoneTaskList() {
        // Move 1 - to shooting position
        DriveConstraints constraints = new DriveConstraints(20.0, 10.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);

        Pose2d p0 = getProfilePose("START");
        Pose2d p1 = getProfilePose("SHOOT");
        Trajectory trjShoot = robotHardware.mecanumDrive.trajectoryBuilder(p0, constraints)
                .splineTo(p1.vec(), p1.getHeading())
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trjShoot);
        ParallelComboTask par1 = new ParallelComboTask();
        par1.addTask(moveTask1);
        par1.addTask(new ShooterMotorTask(robotHardware, robotProfile, true));
        taskList.add(par1);

        // Shooting action
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, true));

        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, false));

        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 10));

        Pose2d p2 = getProfilePose("A-1");
        Trajectory trjWob = robotHardware.mecanumDrive.trajectoryBuilder(p1, constraints)
                .splineToSplineHeading(p2, p2.getHeading())
                .build();
        SplineMoveTask moveTask2 = new SplineMoveTask(robotHardware.mecanumDrive, trjWob);
        taskList.add(moveTask2);
        // Wobble goal dropping
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 1000));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 200));
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
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 1500));

        Pose2d p4 = getProfilePose("A-2");
        Trajectory trjWob2 = robotHardware.mecanumDrive.trajectoryBuilder(p3, true)
                .splineToSplineHeading(p4, p4.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask4 = new SplineMoveTask(robotHardware.mecanumDrive, trjWob2);
        taskList.add(moveTask4);
        // Drop wobble 2
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 500));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 500));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 1500));
//        taskList.add(new GrabberTask(robotHardware, robotProfile, false, 500));
        Pose2d p5 = getProfilePose("PARKING");
        Trajectory trjPark = robotHardware.mecanumDrive.trajectoryBuilder(p4, true)
                .splineToSplineHeading(p5, p5.getHeading())
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjPark));

    }
}

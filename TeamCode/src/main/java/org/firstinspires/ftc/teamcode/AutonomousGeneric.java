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
        DriveConstraints moveFast = new DriveConstraints(40.0, 30.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints extraFastConstraints = new DriveConstraints(100.0, 100.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);

        Logger.logFile("Preparing QUAD Task List");
        Pose2d p0 = getProfilePose("START_STATE");
        Pose2d p1a = getProfilePose("FIRST_SHOOT_STATE");
        Trajectory trjShoot1 = robotHardware.mecanumDrive.trajectoryBuilder(p0, constraints)
                .splineToSplineHeading(p1a, p1a.getHeading(), moveFast)
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trjShoot1);
        ParallelComboTask par1 = new ParallelComboTask();
        par1.addTask(moveTask1);
        par1.addTask(new ShooterMotorTask(robotHardware, robotProfile, true, -1230));
        par1.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        taskList.add(par1);

        // Shooting action
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));

        Pose2d p1b =  getProfilePose("QUAD_COLLISION_STATE");
        Trajectory trjCollision = robotHardware.mecanumDrive.trajectoryBuilder(p1a, extraFastConstraints)
                                   .splineToSplineHeading(p1b, p1b.getHeading(), extraFastConstraints)
                                    .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjCollision));

        taskList.add(new IntakeMotorTask(robotHardware, robotProfile, IntakeMotorTask.IntakeMode.NORMAL));

        //2nd shoot - move forward and pick up rings and shoot 2 times
        Pose2d p2 =  getProfilePose("RINGS_PICK1_STATE");
        Trajectory trjPickupRings = robotHardware.mecanumDrive.trajectoryBuilder(p1b, moveFast)
                .splineToSplineHeading(p2, p2.getHeading(), constraints)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjPickupRings));
        taskList.add(new RobotSleep(1000));

        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));

        //3rd shoot - move forward and pick up rings and shoot 2 times
        Pose2d p3 =  getProfilePose("RINGS_PICK2_STATE");
        Trajectory trjPickupLastRings = robotHardware.mecanumDrive.trajectoryBuilder(p2, moveFast)
                                        .splineToSplineHeading(p3, p3.getHeading(), moveFast)
                                        .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjPickupLastRings));
        taskList.add(new RobotSleep(800));

        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        Logger.logFile("After shoot all");

        ParallelComboTask wobbleDelivery = new ParallelComboTask();
        wobbleDelivery.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 30));
        wobbleDelivery.addTask(new IntakeMotorTask(robotHardware, robotProfile,IntakeMotorTask.IntakeMode.STOP));
        Pose2d p4 = getProfilePose("C_1_STATE");
        Trajectory trjWob = robotHardware.mecanumDrive.trajectoryBuilder(p3, moveFast)
                            .splineToSplineHeading(p4, p4.getHeading(), extraFastConstraints)
                            .build();
        wobbleDelivery.addTask(new SplineMoveTask(robotHardware.mecanumDrive, trjWob));
        taskList.add(wobbleDelivery);

        //Wobble goal dropping
         taskList.add(new GrabberTask(robotHardware, robotProfile, true, 40));
         Logger.logFile("delivered 1st wobble goal");

        ParallelComboTask prepareWobblePickUp = new ParallelComboTask();
        prepareWobblePickUp.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));
        prepareWobblePickUp.addTask(new ShooterMotorTask(robotHardware, robotProfile, false));
        prepareWobblePickUp.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 200));
        taskList.add(prepareWobblePickUp);

        Pose2d p5a = getProfilePose("WG2_PICKPre_STATE");
        Pose2d p5b = getProfilePose("WG2_PICK_STATE");
        Trajectory trjWob2a = robotHardware.mecanumDrive.trajectoryBuilder(p4, true)
                              .splineToSplineHeading(p5a, p5a.getHeading(), constraints)
                              .splineToSplineHeading(p5b, p5b.getHeading(), constraints)
                              .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjWob2a));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 30));

        taskList.add(new AutoWobbleGoalPickUpTask(robotHardware,robotProfile));
        taskList.add(new GrabberTask(robotHardware, robotProfile, false, 200));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 30));

        Pose2d p7 = getProfilePose("C_WG2_DELIVER_STATE");
        Trajectory trjWob2b = robotHardware.mecanumDrive.trajectoryBuilder(p5b, true)
                .splineToSplineHeading(p7, p7.getHeading(), moveFast)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjWob2b));

        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 10));
        Logger.logFile("delivered the 2nd wobble goal");

        Pose2d p8 = getProfilePose("C_PARKING");
        Trajectory trjParking = robotHardware.mecanumDrive.trajectoryBuilder(p7, false)
                .splineToSplineHeading(p8, p8.getHeading(), extraFastConstraints)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjParking));
        Logger.logFile("parking at the line");
    }

    void prepareSingleTaskList() {
        DriveConstraints constraints = new DriveConstraints(20.0, 10.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints moveExtraFast = new DriveConstraints(100.0, 100.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);

        // move to shoot
        Pose2d p0 = getProfilePose("START_STATE");
        Pose2d p1 = getProfilePose("AUTOB-SHOOT-POWER-BAR-3");
        Trajectory trjShoot1 = robotHardware.mecanumDrive.trajectoryBuilder(p0, constraints)
                                .splineToSplineHeading(p1, p1.getHeading(), moveFast)
                                .build();
        ParallelComboTask par1 = new ParallelComboTask();
        par1.addTask(new SplineMoveTask(robotHardware.mecanumDrive, trjShoot1));
        par1.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        par1.addTask(new ShooterMotorTask(robotHardware, robotProfile, true, robotProfile.hardwareSpec.autonomousShootVelocity));
        taskList.add(par1);
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));

        Pose2d p2 = getProfilePose("AUTOB-SHOOT-POWER-BAR-2");
        Trajectory trjShoot2 = robotHardware.getMecanumDrive().trajectoryBuilder(p1, constraints)
                                .lineToLinearHeading(p2, constraints)
                                .build();
        taskList.add(new SplineMoveTask(robotHardware.getMecanumDrive(), trjShoot2));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));

        Pose2d p3 = getProfilePose("AUTOB-SHOOT-POWER-BAR-1");
        Trajectory trjShoot3 = robotHardware.getMecanumDrive().trajectoryBuilder(p2, constraints)
                                .lineToLinearHeading(p3, moveFast)
                                .build();
        taskList.add(new SplineMoveTask(robotHardware.getMecanumDrive(), trjShoot3));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(100));

        ParallelComboTask prePickUpRing4 = new ParallelComboTask();
        prePickUpRing4.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));
        prePickUpRing4.addTask(new IntakeMotorTask(robotHardware, robotProfile, IntakeMotorTask.IntakeMode.NORMAL));
        taskList.add(prePickUpRing4);

        Pose2d p4a = new Pose2d(-30,-40,Math.toRadians(-15));
        Trajectory trjPickupRingsa = robotHardware.mecanumDrive.trajectoryBuilder(p3, moveFast)
                                     .splineToLinearHeading(p4a, p4a.getHeading())
                                      .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjPickupRingsa));
        taskList.add(new RobotSleep(500));

        Pose2d p4b = new Pose2d(-20,-33,Math.toRadians(-6));
        Trajectory trjPickupRingsb = robotHardware.mecanumDrive.trajectoryBuilder(p4a, moveFast)
                .splineToLinearHeading(p4b, p4b.getHeading())
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjPickupRingsb));
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));

        ParallelComboTask preDropWG1 = new ParallelComboTask();
        preDropWG1.addTask(new IntakeMotorTask(robotHardware, robotProfile,IntakeMotorTask.IntakeMode.STOP));
        preDropWG1.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));
        preDropWG1.addTask(new ShooterMotorTask(robotHardware, robotProfile, false));
        taskList.add(preDropWG1);

        // move to drop wobble 1
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 10));
        Pose2d p5 = getProfilePose("B-1");;
        Trajectory trjWob = robotHardware.mecanumDrive.trajectoryBuilder(p4b, moveFast)
                            .splineToSplineHeading(p5, p5.getHeading(), moveExtraFast)
                            .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjWob));

        // Wobble goal dropping
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 10));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 100));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 10));

        // Move to grab wobble 2
        Pose2d p7 = getProfilePose("B-WB2");
        Trajectory pickUp = robotHardware.mecanumDrive.trajectoryBuilder(p5, true)
                             .splineToSplineHeading(p7, p7.getHeading(), constraints)
                             .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, pickUp));
        taskList.add(new RobotSleep(100));

        // Auto Grab wobble goal 2
        taskList.add(new AutoWobbleGoalPickUpTask(robotHardware,robotProfile));
        taskList.add(new RobotSleep(1000));
        taskList.add(new GrabberTask(robotHardware, robotProfile, false, 200));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 100));

        // Move to drop wobble 2
        Pose2d p8 = getProfilePose("B-2");
        Trajectory trjWob2 = robotHardware.mecanumDrive.trajectoryBuilder(p7, true)
                .splineToSplineHeading(p8, p8.getHeading(), moveExtraFast)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjWob2));

        // Drop wobble 2
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 10));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 100));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 10));

        // move to parking
        Trajectory trjPark = robotHardware.mecanumDrive.trajectoryBuilder(p8, moveFast)
                .back(5, moveExtraFast)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjPark));
    }

    void prepareNoneTaskList() {
        DriveConstraints constraints = new DriveConstraints(20.0, 10.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);

        Pose2d p0 = getProfilePose("START_STATE");
        Pose2d p1 = getProfilePose("AUTOA-SHOOT-POWER-BAR-3");
        Trajectory trjShoot1 = robotHardware.mecanumDrive.trajectoryBuilder(p0, moveFast)
                .splineToSplineHeading(p1, p1.getHeading(), moveFast)
                .build();
        ParallelComboTask par1 = new ParallelComboTask();
        par1.addTask(new SplineMoveTask(robotHardware.mecanumDrive, trjShoot1));
        par1.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        par1.addTask(new ShooterMotorTask(robotHardware, robotProfile, true, robotProfile.hardwareSpec.shootBarVelocity));
        taskList.add(par1);
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));

        Pose2d p2 = getProfilePose("AUTOA-SHOOT-POWER-BAR-2");
        Trajectory trjShoot2 = robotHardware.getMecanumDrive().trajectoryBuilder(p1, constraints)
                .lineToLinearHeading(p2, constraints)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.getMecanumDrive(), trjShoot2));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));

        Pose2d p3 = getProfilePose("AUTOA-SHOOT-POWER-BAR-1");
        Trajectory trjShoot3 = robotHardware.getMecanumDrive().trajectoryBuilder(p2, constraints)
                .lineToLinearHeading(p3, constraints)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.getMecanumDrive(), trjShoot3));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, false));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 10));
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));

        Pose2d p4 = getProfilePose("A-1");
        Trajectory trjWob = robotHardware.mecanumDrive.trajectoryBuilder(p3, constraints)
                .splineToSplineHeading(p4, p4.getHeading())
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjWob));

        // Wobble goal dropping
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 200));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 200));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 200));

        // Move to Wobble 2
        Pose2d p5 = getProfilePose("A-WB2");
        Trajectory trjPickup = robotHardware.mecanumDrive.trajectoryBuilder(p4, true)
                .splineToSplineHeading(p5, p5.getHeading(), constraints)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjPickup));

        // Auto Grab wobble goal 2
        taskList.add(new AutoWobbleGoalPickUpTask(robotHardware,robotProfile));
        taskList.add(new RobotSleep(1000));
        taskList.add(new GrabberTask(robotHardware, robotProfile, false, 300));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 100));

        Pose2d p6b = getProfilePose("A-2");
        Trajectory trjWob2 = robotHardware.mecanumDrive.trajectoryBuilder(p5, true)
                .splineToSplineHeading(p6b, p6b.getHeading(), constraints)
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjWob2));

        // Drop wobble 2
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.GRAB, 200));
        taskList.add(new GrabberTask(robotHardware, robotProfile, true, 300));
        taskList.add(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 200));

        Pose2d p7 = getProfilePose("PARKING");
        Trajectory trjPark = robotHardware.mecanumDrive.trajectoryBuilder(p6b, true)
                .splineToSplineHeading(p7, p7.getHeading())
                .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjPark));
    }
}

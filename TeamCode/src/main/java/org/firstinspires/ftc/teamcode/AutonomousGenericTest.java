package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;


import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;


/**
 * 2019.10.26
 * Created by Ian Q.
 */
@TeleOp(name="AutonomousTest", group="Test")
public class AutonomousGenericTest extends LinearOpMode {

    RobotHardware robotHardware;
    RobotProfile robotProfile;
    RobotVision robotVision;

    ArrayList<RobotControl> taskList;

    long loopCount = 0;
    int countTasks = 0;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    String startPosStr = "BLUE";

    public void initRobot() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }
        Logger.init();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        robotHardware.resetLiftPositionAutonomous();

        Logger.logFile("Init completed");
    }

    @Override
    public void runOpMode() {

        initRobot();
        robotHardware.setMotorStopBrake(false); // so we can adjust the robot
        robotHardware.initRobotVision();
        robotVision = robotHardware.getRobotVision();
        robotVision.initRearCamera(false);  //isRed boolean

        //robotHardware.getTrackingWheelLocalizer().setPoseEstimate(new Pose2d(-66, -33, 0));
        //robotHardware.getLocalizer().update();
        //robotHardware.getMecanumDrive().setPoseEstimate(getProfilePose("START"));
        long loopStart = System.currentTimeMillis();
        long loopCnt = 0;
        while (!isStarted()) {
            //RobotVision.AutonomousGoal goal = robotHardware.getRobotVision().getAutonomousRecognition(false);
            //telemetry.addData("goal",goal);
            robotHardware.getLocalizer().update();
            Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
            loopCnt++;
            if (loopCnt%100==0) {
                telemetry.addData("CurrPose", currPose);
                telemetry.addData("T265 CFD:",  ((RealSenseLocalizer)robotHardware.getLocalizer()).getT265Confidence());
                telemetry.addData("LoopTPS:", (loopCnt * 1000 / (System.currentTimeMillis() - loopStart)));
                telemetry.addData("RearCameraOpen", robotVision.isRearCameraOpened());
                telemetry.update();
            }
        }

        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        Logger.logFile("Recognition Result: " + robotVision.getAutonomousRecognition(startPosStr.contains("BULE") ? false : true));
        taskList = new ArrayList<RobotControl>();
//        setupTaskList3();
//        setupTaskList1();
//        setupRedDuckTasks();
//        testCentralDeliverShippingHub();
        testDepotPickUp();
//        testDeliverSharingHub_T265();
        robotHardware.setMotorStopBrake(true);
        TaskReporter.report(taskList);
        Logger.logFile("Task list items: " + taskList.size());
        Logger.flushToFile();

        if (taskList.size()>0) {
            Logger.logFile("Task Prepare " + taskList.get(0));
            taskList.get(0).prepare();
        }
        // run until the end of the match (driver presses STOP)
        // run until the end of the match (driver presses STOP)
        long startTime = System.currentTimeMillis();
        int cnt = 100;
        double veloSum = 0;
        robotVision.startRearCamera();
        Logger.logFile("Main Task Loop started");

        while (opModeIsActive()) {
            loopCount++;
            robotHardware.getBulkData1();
            robotHardware.getBulkData2();
            robotHardware.getLocalizer().update();
            //Logger.logFile("Pose:" + robotHardware.getLocalizer().getPoseEstimate());
            //Logger.logFile("Velocity:" + robotHardware.getLocalizer().getPoseVelocity());
            try {
                Logger.flushToFile();
            }
            catch (Exception ex) {
            }
            /*Specific test for motor velocity */
// append to shooting velocity csv file
            /* End Testing code */
            if (taskList.size() > 0) {
                taskList.get(0).execute();
                if (taskList.get(0).isDone()) {
                    Logger.logFile("MainTaskComplete: " + taskList.get(0) + " Pose:" + robotHardware.getLocalizer().getPoseEstimate());
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
        robotVision.stopRearCamera();
        try {
            Logger.flushToFile();
        }
        catch (Exception ex) {
        }
    }

    void setupTaskList1() {
//        DriveConstraints constraints = new DriveConstraints(5.0, 5.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
//        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        velConstraint = getVelocityConstraint(15, 10, TRACK_WIDTH);
        accelConstraint = getAccelerationConstraint(10);
        // move to shoot
        Pose2d p0 = new Pose2d(0,0,0);
        Pose2d p1 = new Pose2d(20, 0, 0);
        //Pose2d p2 = new Pose2d(10,5,0);
        Trajectory trj = robotHardware.mecanumDrive.trajectoryBuilder(p0)
                .lineTo(p1.vec(), velConstraint, accelConstraint)
                //.splineToSplineHeading(p2, p2.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trj);
        taskList.add(new RobotSleep(300));
        taskList.add(moveTask1);
        taskList.add(new RobotSleep(300));
    }

    void setupTaskList2() {
        PIDMecanumMoveTask pmm1 = new PIDMecanumMoveTask(robotHardware,robotProfile);
        pmm1.setPower(0.5);
        pmm1.setPath(new Pose2d(0,0,0), new Pose2d(20, 0, 0));

        taskList.add(pmm1);

        MecanumRotateMoveTask mrm1 = new MecanumRotateMoveTask(robotHardware, robotProfile);
        mrm1.setRotateHeading(new Pose2d(20,0,0), new Pose2d(20, 0, Math.PI/2));
        mrm1.setPower(0.3);
        taskList.add(mrm1);

        PIDMecanumMoveTask pmm2 = new PIDMecanumMoveTask(robotHardware, robotProfile);
        pmm2.setPath(new Pose2d(20,0,Math.PI/2), new Pose2d(20, -20, Math.PI/2));
        pmm2.setPower(0.3);
        taskList.add(pmm2);

        MecanumRotateMoveTask mrm2 = new MecanumRotateMoveTask(robotHardware, robotProfile);
        mrm2.setRotateHeading(new Pose2d(20,-20,Math.PI/2), new Pose2d(30, -20, 0));
        mrm2.setPower(0.3);
        taskList.add(mrm2);

//        PIDMecanumMoveTask pmm2 = new PIDMecanumMoveTask(robotHardware,robotProfile);
//        pmm2.setPower(0.3);
//        pmm2.setPath(new Pose2d(1,0,Math.PI/2), new Pose2d(10, -10, Math.PI/2));
//
//        taskList.add(pmm2);
    }

    void setupTaskList3() {
        MecanumRotateMoveTask mrm1 = new MecanumRotateMoveTask(robotHardware, robotProfile);
        mrm1.setRotateHeading(new Pose2d(00, 0, 0), new Pose2d(-15, -15, -Math.PI / 3));
        mrm1.setPower(0.3);
        taskList.add(mrm1);
        taskList.add(new RobotSleep(1000));
        MecanumRotateMoveTask mrm2 = new MecanumRotateMoveTask(robotHardware, robotProfile);
        mrm2.setRotateHeading(new Pose2d(-15, -15, -Math.PI / 3), new Pose2d(00, 0, 0));
        mrm2.setPower(0.3);
        taskList.add(mrm2);
    }

    void testCentralDeliverShippingHub(){
        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(15, 15, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(100, 100, 10.25);
        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(30, 30, 10.25);
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((15));
        SampleMecanumDrive drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();
        String startPosStr = "BLUE";
        RobotHardware.LiftPosition targetLiftLevel = RobotHardware.LiftPosition.TOP;
        robotHardware.resetImu();
        robotHardware.getLocalizer().setPoseEstimate(robotProfile.getProfilePose(startPosStr + "_AUTO_HUB_DELIVERY_START"));
        Pose2d startPos = robotProfile.getProfilePose(startPosStr + "_AUTO_HUB_DELIVERY_START");
        Pose2d warehousePos = robotProfile.getProfilePose(startPosStr + "_WAREHOUSE_PICKUP");
        Pose2d preHubPos_0 = robotProfile.getProfilePose(startPosStr + "_CENTRAL_PRE_HUB_0");
        Pose2d preHubPos_1 = robotProfile.getProfilePose(startPosStr + "_CENTRAL_PRE_HUB_1");
        Pose2d hubPos = robotProfile.getProfilePose(startPosStr + "_CENTRAL_HUB");

        ParallelComboTask par0 = new ParallelComboTask();
        Trajectory traj0 =  drive.trajectoryBuilder(startPos)
                            .lineTo(warehousePos.vec(), fastVelConstraints, accConstraint)
                            .build();
        par0.addTask(new SplineMoveTask(drive, traj0));
        par0.addTask(new IntakeTask(robotHardware, robotProfile));
        taskList.add(par0);

        Trajectory traj1 =  drive.trajectoryBuilder(warehousePos, true)
                .splineTo(preHubPos_0.vec(), preHubPos_0.getHeading() + Math.PI, fastVelConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj1));

        Trajectory traj2 =  drive.trajectoryBuilder(preHubPos_0, true)
                            .splineTo(preHubPos_1.vec(), preHubPos_1.getHeading()+Math.PI, fastVelConstraints, accConstraint)
                            .build();
        taskList.add(new SplineMoveTask(drive, traj2));

        taskList.add(new LiftBucketTask(robotHardware, robotProfile, targetLiftLevel));

        Trajectory traj3 =  drive.trajectoryBuilder(preHubPos_1, true)
                .strafeTo(hubPos.vec(), fastVelConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj3));

        taskList.add(new AutoHubApproachTask(robotHardware, robotProfile));

        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));
        taskList.add(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ONE));
    }

    void testDepotPickUp(){
        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(25, 25, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(50, 50, 10.25);
        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(10, 10, 10.25);
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((40));
        TrajectoryAccelerationConstraint slowAccConstraint = SampleMecanumDrive.getAccelerationConstraint((25));
        SampleMecanumDrive drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();
        String parking = "WALL";
        String startPosStr = "RED_DEPOT";

        Pose2d warehousePickupPos_0 = robotProfile.getProfilePose(startPosStr + "_PICKUPWALL_0");
        Pose2d warehousePickupPos_1 = robotProfile.getProfilePose(startPosStr + "_PICKUPWALL_1");
        Pose2d warehousePickupPos_2 = robotProfile.getProfilePose(startPosStr + "_PICKUPWALL_2");
        Pose2d prePickPos = robotProfile.getProfilePose(startPosStr + "_PREWALL");;
        Pose2d preParkPos = null, parkPos=null;
        Pose2d startPos = robotProfile.getProfilePose(startPosStr + "_START");


        if (parking.contains("WALL")) {
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PREWALL");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARKWALL");
        }

        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                            .splineTo(prePickPos.vec(), prePickPos.getHeading())
                            .build();
        taskList.add(new SplineMoveTask(drive, traj1));

        Trajectory traj4 = drive.trajectoryBuilder(prePickPos)
                .splineTo(warehousePickupPos_0.vec(), warehousePickupPos_0.getHeading(), velConstraints, accConstraint)
                .splineTo(warehousePickupPos_2.vec(), warehousePickupPos_2.getHeading(), slowVelConstraints, slowAccConstraint)
                .build();
        ParallelComboIntakeMovePriorityTask par4 = new ParallelComboIntakeMovePriorityTask();
        par4.addTask(new AutoIntakeSplineMoveTask(traj4, robotHardware));
        par4.addTask(new AutoIntakeTask(robotHardware, robotProfile, 6000));
        taskList.add(par4);

        taskList.add(new InstantaneousPostionTrajectoryTask(robotHardware, prePickPos, true));


    }

    void testDeliverSharingHub_T265() {
        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(25, 25, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(100, 100, 10.25);
        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(10, 10, 10.25);
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((15));
        SampleMecanumDrive drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();
//        String startPosStr = "BLUE";
        robotHardware.getLocalizer().setPoseEstimate(robotProfile.getProfilePose(startPosStr + "_SHARE_START"));

        Pose2d startPos = robotProfile.getProfilePose(startPosStr + "_SHARE_START");
        Pose2d intakePos_0 = robotProfile.getProfilePose(startPosStr + "_SHARE_INTAKE_0");
        Pose2d intakePos_1 = robotProfile.getProfilePose(startPosStr + "_SHARE_INTAKE_1");
        Pose2d intakePos_2 = robotProfile.getProfilePose(startPosStr + "_SHARE_INTAKE_2");
        Pose2d intakePos_3 = robotProfile.getProfilePose(startPosStr + "_SHARE_INTAKE_3");
        Pose2d preHubPos_1 = robotProfile.getProfilePose(startPosStr + "_SHARE_PRE_HUB_1");
        Pose2d hubPos = robotProfile.getProfilePose(startPosStr + "_SHARE_HUB");
        Pose2d afterHubPos_0 = robotProfile.getProfilePose(startPosStr + "_SHARE_AFTER_HUB");

        Trajectory traj1a = drive.trajectoryBuilder(startPos)
                .splineToLinearHeading(intakePos_0, intakePos_0.getHeading(), velConstraints, accConstraint)
                .build();
        ParallelComboTask par1a = new ParallelComboTask();
        par1a.addTask(new SplineMoveTask(drive, traj1a));
        par1a.addTask(new AutoIntakeTask(robotHardware, robotProfile, 5000));
        taskList.add(par1a);

        Trajectory traj1b = drive.trajectoryBuilder(intakePos_0, true)
                            .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading() + Math.PI, velConstraints, accConstraint)
                            .build();
        taskList.add(new SplineMoveTask(drive, traj1b));
        taskList.add(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.BOTTOM));

        Trajectory traj1c = drive.trajectoryBuilder(preHubPos_1, true)
                .splineToLinearHeading(hubPos, hubPos.getHeading()+Math.PI,  velConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj1c));

        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

        Trajectory traj1d = drive.trajectoryBuilder(hubPos)
                .splineToLinearHeading(afterHubPos_0, afterHubPos_0.getHeading(),  velConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj1d));

        Trajectory traj1e = drive.trajectoryBuilder(afterHubPos_0)
                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading())
                .build();
        taskList.add(new SplineMoveTask(drive, traj1e));

//        2nd pickup
        ParallelComboTask par2a = new ParallelComboTask();
        Trajectory traj2a = drive.trajectoryBuilder(preHubPos_1)
                .splineToLinearHeading(intakePos_1, intakePos_1.getHeading())
                .build();
        par2a.addTask(new SplineMoveTask(drive, traj2a));
        par2a.addTask(new AutoIntakeTask(robotHardware, robotProfile, 5000));
        taskList.add(par2a);

        Trajectory traj2b = drive.trajectoryBuilder(intakePos_1, true)
                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading() + Math.PI, velConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj2b));
        taskList.add(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.BOTTOM));

        Trajectory traj2c = drive.trajectoryBuilder(preHubPos_1, true)
                .splineToLinearHeading(hubPos, hubPos.getHeading()+Math.PI,  velConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj2c));

        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

        Trajectory traj2d = drive.trajectoryBuilder(hubPos)
                .splineToLinearHeading(afterHubPos_0, afterHubPos_0.getHeading(),  velConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj2d));

        Trajectory traj2e = drive.trajectoryBuilder(afterHubPos_0, true)
                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading())
                .build();
        taskList.add(new SplineMoveTask(drive, traj2e));

        //3rd pick up
        Trajectory traj3a = drive.trajectoryBuilder(preHubPos_1)
                .splineToLinearHeading(intakePos_2, intakePos_2.getHeading())
                .build();
        ParallelComboTask par3a = new ParallelComboTask();
        par3a.addTask(new SplineMoveTask(drive, traj3a));
        par3a.addTask(new AutoIntakeTask(robotHardware, robotProfile, 5000));
        taskList.add(par3a);

        Trajectory traj3b = drive.trajectoryBuilder(intakePos_2, true)
                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading() + Math.PI, velConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj3b));
        taskList.add(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.BOTTOM));

        Trajectory traj3c = drive.trajectoryBuilder(preHubPos_1, true)
                .splineToLinearHeading(hubPos, hubPos.getHeading()+Math.PI,  velConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj3c));

        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

        Trajectory traj3d = drive.trajectoryBuilder(hubPos)
                .splineToLinearHeading(afterHubPos_0, afterHubPos_0.getHeading(),  velConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj3d));

        Trajectory traj3e = drive.trajectoryBuilder(afterHubPos_0, true)
                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading())
                .build();
        taskList.add(new SplineMoveTask(drive, traj3e));

        //4th pick up
        ParallelComboTask par4a = new ParallelComboTask();
        Trajectory traj4a = drive.trajectoryBuilder(preHubPos_1)
                .splineToLinearHeading(intakePos_3, intakePos_3.getHeading())
                .build();
        par4a.addTask(new SplineMoveTask(drive, traj4a));
        par4a.addTask(new AutoIntakeTask(robotHardware, robotProfile, 5000));
        taskList.add(par4a);

        Trajectory traj4b = drive.trajectoryBuilder(intakePos_3, true)
                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading() + Math.PI, velConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj4b));
        taskList.add(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.MIDDLE));

        Trajectory traj4c = drive.trajectoryBuilder(preHubPos_1, true)
                .splineToLinearHeading(hubPos, hubPos.getHeading()+Math.PI,  velConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj4c));

        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

        Trajectory traj4d = drive.trajectoryBuilder(hubPos)
                .splineToLinearHeading(afterHubPos_0, afterHubPos_0.getHeading(),  velConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj4d));

        Trajectory traj4e = drive.trajectoryBuilder(afterHubPos_0)
                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading())
                .build();
        taskList.add(new SplineMoveTask(drive, traj4e));

        Trajectory traj4f = drive.trajectoryBuilder(preHubPos_1)
                .splineToLinearHeading(startPos, startPos.getHeading(),  velConstraints, accConstraint)
                .build();
        taskList.add(new SplineMoveTask(drive, traj4f));
    }

    void setupRedDuckTasks() {
        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(15, 15, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(25, 25, 10.25);
        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(11, 11, 10.25);
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((15));
        SampleMecanumDrive drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();
        String startPosStr = "RED_DUCK";
        String parking = "CENTRAL";

        Pose2d startPos = robotProfile.getProfilePose(startPosStr + "_START");
        Pose2d preCarouselPos = robotProfile.getProfilePose(startPosStr + "_PRECAROUSEL");
        Pose2d duckSpinPos = robotProfile.getProfilePose(startPosStr + "_CAROUSEL");
        Pose2d preHubPos = robotProfile.getProfilePose(startPosStr + "_PREHUB");
        Pose2d prehub1 = robotProfile.getProfilePose(startPosStr + "_PREHUB_1");
        Pose2d prehub2 = robotProfile.getProfilePose(startPosStr + "_PREHUB_2");
        Pose2d hubPos = robotProfile.getProfilePose(startPosStr + "_HUB");
        Pose2d preParkPos_0 = robotProfile.getProfilePose(startPosStr + "_PRE_PARK_0");
        Pose2d preParkPos_1 = robotProfile.getProfilePose(startPosStr + "_PRE_PARK_1");
        Pose2d parkWay1 = robotProfile.getProfilePose(startPosStr + "_PARKWAY_1");
        Pose2d preParkPos, parkPos, hubEstimatePos;
        RobotHardware.LiftPosition targetLiftLevel = RobotHardware.LiftPosition.TOP;
        int delay_parking = 0;


        robotHardware.getLocalizer().setPoseEstimate(startPos);

        if (parking.endsWith("WALL")) {
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PREWALL");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARKWALL");
        }
        else if(parking.endsWith("CENTRAL")) {
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PRE_CENTRAL");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARK_CENTRAL");
        }
        else { //if(parking.contains("STORAGE")){
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PREPARKSTORAGE");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARKSTORAGE");
        }
        // Move to Duck Carousel
        Trajectory traj0 = drive.trajectoryBuilder(startPos)
                .strafeTo(preCarouselPos.vec())
                .build();
        taskList.add(new SplineMoveTask(drive, traj0));

        Trajectory traj1 = drive.trajectoryBuilder(preCarouselPos)
                .splineTo(duckSpinPos.vec(), duckSpinPos.getHeading(),slowVelConstraints, accConstraint)
                .forward(1)
                .build();
        taskList.add(new SplineMoveTask(drive, traj1));
        taskList.add(new DuckCarouselSpinTask(robotHardware, startPosStr));

        // Lift Bucket and move to Hub
        ParallelComboTask par1 = new ParallelComboTask();
        Trajectory traj2 = drive.trajectoryBuilder(duckSpinPos, true)
                .splineTo(prehub1.vec(), prehub1.getHeading())
                .splineTo(prehub2.vec(), prehub2.getHeading())
                .splineTo(hubPos.vec(), hubPos.getHeading())
                .build();
        par1.addTask(new SplineMoveTask(drive, traj2));
        par1.addTask(new LiftBucketTask(robotHardware, robotProfile, targetLiftLevel));
        taskList.add(par1);
//        Trajectory traj3 = drive.trajectoryBuilder(preHubPos,true)
//                .splineTo(hubPos.vec(),hubPos.getHeading())
//                .build();
//        taskList.add(new SplineMoveTask(drive, traj3));

        taskList.add(new AutoHubApproachTask(robotHardware, robotProfile));

        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));
        taskList.add(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ONE));
//
        if (delay_parking>0){
            taskList.add(new RobotSleep(delay_parking * 1000));
        }

        if(startPosStr.contains("RED")) {
            hubEstimatePos = new Pose2d(8, 44, Math.PI);
        }else{
            hubEstimatePos = new Pose2d(8, -44, Math.PI);
        }
        if(parking.contains("STORAGE")){
            Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(hubEstimatePos.getX(), hubEstimatePos.getY(), Math.toRadians(180)))
                    .splineTo(preParkPos.vec(), preParkPos.getHeading())
                    .build();
            taskList.add(new SplineMoveTask(drive, traj4));

            Trajectory traj5 = drive.trajectoryBuilder(preParkPos)
                    .strafeTo(parkPos.vec())
                    .build();
            taskList.add(new SplineMoveTask(drive, traj5));
        } else {   //wall or central parking
            Trajectory traj4 = drive.trajectoryBuilder(hubEstimatePos)
                    .splineTo(parkWay1.vec(), parkWay1.getHeading())
                    .splineTo(preParkPos_1.vec(), preParkPos_1.getHeading())
                    .build();
            taskList.add(new SplineMoveTask(drive, traj4));

            Trajectory traj5a = drive.trajectoryBuilder(preParkPos_1)
                    .strafeTo(preParkPos.vec())
                    .build();
            taskList.add(new SplineMoveTask(drive, traj5a));

            Trajectory traj5b = drive.trajectoryBuilder(preParkPos)
                    .splineTo(parkPos.vec(), parkPos.getHeading(), fastVelConstraints, accConstraint)
                    .build();
            taskList.add(new SplineMoveTask(drive, traj5b));
        }
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}

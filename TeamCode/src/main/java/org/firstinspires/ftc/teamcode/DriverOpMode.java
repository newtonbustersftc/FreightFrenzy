package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="Newton DriverOpMode", group="Main")
public class DriverOpMode extends OpMode {
    RobotHardware robotHardware;
    RobotProfile robotProfile;

    Pose2d currPose;
    double fieldHeadingOffset;

    boolean fieldMode;
    int fieldModeSign = -1;  // RED side = 1, BLUE side = -1
    boolean isRedTeam;
    boolean dpadRightPressed = false;
    boolean dpadLeftPressed = false;
    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;
    boolean aPressed = false;
    boolean yPressed = false;
    boolean xPressed = false;
    boolean dpadUpPressed = false;
    boolean dpadDownPressed = false;
    boolean leftTriggerPressed = false;
    boolean rightTriggerPressed = false;
    double imuAngleOffset = 0;

    // DriveThru combos
    SequentialComboTask intakeAndLift, deliverTask,  sharedHubTask;
    RobotControl currentTask = null;
    String startPosStr ;

    @Override
    public void init() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        }
        catch (Exception e) {
            System.out.println(e.getStackTrace());
        }
        fieldMode = true; //robot starts in field orientation

        Logger.init();
        //Obtain the RobotHardware object from factory
        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        //robotHardware.initRobotVision();
        //robotVision = robotHardware.getRobotVision();
        //robotVision.activateNavigationTarget();
        //robotHardware.initLeds();   // need to init everytime
       // robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        //ensure lift is reset at the beginning and the end
        // Based on the Autonomous mode starting position, define the heading offset for field mode
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        if (prefs.getString(START_POS_MODES_PREF, "NONE").startsWith("RED")) {
            fieldModeSign = 1;
            isRedTeam = true;
        }
        else {
            fieldModeSign = -1;
        }
        if (prefs.getString(START_POS_MODES_PREF, "NONE").contains("DUCK")) {
            imuAngleOffset = Math.PI;
        }
        Logger.logFile("DriverOpMode: " + prefs.getString(START_POS_MODES_PREF, "NONE"));
        Logger.logFile("IMU Offset is " + Math.toDegrees(imuAngleOffset));
        Logger.logFile("Current IMU Angle " + Math.toDegrees(robotHardware.getImuHeading()));

        //robotHardware.getRobotVision().initRearCamera(isRedTeam);

        startPosStr = prefs.getString(START_POS_MODES_PREF, "NONE").contains("BLUE") ? "BLUE" : "RED";
        setupCombos();
        if (robotHardware.getCurrLiftPos()== RobotHardware.LiftPosition.NOT_INIT) {
            currentTask = new ResetLiftPositionDriverOpModeTask(robotHardware);
            currentTask.prepare();
        }
    }

    /**
     * Main loop for the code
     */
    @Override
    public void loop() {
        //Read values from the control hub
        robotHardware.getBulkData1();
        //Read values from the expansion hub
        robotHardware.getBulkData2();

        //currPose = new Pose2d(0,0,0);   // for now
        //Handling autonomous task loop
        if (currentTask != null) {
            robotHardware.setLed1(true);
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                currentTask.cleanUp();
                currentTask = null;
            }
            else {
                currentTask.execute();
                if (currentTask.isDone()) {
                    currentTask.cleanUp();
                    Logger.logFile("TaskComplete: " + currentTask);
                    currentTask = null;
                }
            }
        }
        else {
            robotHardware.setLed1(false);
        }

        handleMovement();

        telemetry.addData("Heading", Math.toDegrees(robotHardware.getImuHeading()));

        handleIntake();

        if (gamepad1.left_bumper && !leftBumperPressed) {
            robotHardware.liftUp();
        }
        else if (gamepad1.right_bumper && !rightBumperPressed) {
            robotHardware.liftDown();
        }
        leftBumperPressed = gamepad1.left_bumper;
        rightBumperPressed = gamepad1.right_bumper;

        if (!xPressed && gamepad1.x) {
            Logger.logFile("field red=" + fieldModeSign);
            if (fieldModeSign == 1) //red
                robotHardware.startDuck(1);
            else
                robotHardware.startDuck(-1);
            xPressed = true;
        }
        if (!gamepad1.x) {
            robotHardware.stopDuck();
            xPressed = false;
        }

        if(gamepad1.b && gamepad1.right_bumper){
            robotHardware.setLiftPosition(RobotHardware.LiftPosition.NOT_INIT);
            currentTask = new ResetLiftPositionDriverOpModeTask(robotHardware);
            currentTask.prepare();
        }


        if (gamepad1.y) {
            //robotHardware.openBoxFlap();
            currentTask = deliverTask;
            deliverTask.prepare();
        }
//        else {
//            robotHardware.closeBoxFlap();
//        }
    }

    @Override
    public void stop() {
        robotHardware.stopAll();
        try {
            Logger.logFile("DriverOpMode stop() called");
            //robotVision.deactivateNavigationTarget();
            Logger.flushToFile();
        }
        catch (Exception e) {
            System.out.println(e.getStackTrace());
        }
    }

    /**
     * Joystick Driving Controls
     * Left bumper for slow motion
     * Left trigger to enable field mode, right trigger to enable robot-oriented mode
     */
    private void handleMovement() {
        double turn = gamepad1.right_stick_x / 2;
        double power = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double padAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) + Math.PI / 2;

        double movAngle;

        robotHardware.setLed2(fieldMode);
        if (fieldMode) {
            movAngle = padAngle + ((isRedTeam) ? Math.PI / 2 : -Math.PI / 2) - robotHardware.getImuHeading()-imuAngleOffset;
        } else {
            movAngle = padAngle;
        }
        if (gamepad1.left_trigger > 0) {
            power = power / 3;
            turn = turn / 3;
        }
        robotHardware.mecanumDrive2(power, movAngle, turn);

        // toggle field mode on/off.
        // Driver 1: dpad down - enable; dpad right - disable
        if (gamepad1.dpad_down) {
            fieldMode = true;
        } else if (gamepad1.dpad_right) {
            fieldMode = false;  //good luck driving
        }
        if(gamepad1.share){
            robotHardware.resetImu();
            imuAngleOffset = 0;
            fieldMode = true;
        }

//        if (gamepad1.left_trigger > 0 && gamepad1.dpad_left) {
//            String startPosStr = START_POS_MODES_PREF.contains("BLUE") ? "BLUE" : "RED";
//            robotHardware.getLocalizer().setPoseEstimate(robotProfile.getProfilePose(startPosStr + "_SHARE_START"));
//            currentTask = sharedHubTask;
//            sharedHubTask.prepare();
//        }

    }

    /**
     * Uses up  down keypad to switch to different modes between shooting, stop, reverse, and intake
     */
    private void handleIntake() {
        if (gamepad1.right_trigger > 0 && !rightTriggerPressed) {
            currentTask = new DriverAutoIntakeTask(robotHardware, robotProfile, gamepad1);
            currentTask.prepare();
            //robotHardware.startIntake();
        }
        rightTriggerPressed = (gamepad1.right_trigger>0);

        if (!aPressed && gamepad1.a) {
//            if (currentTask == intakeAndLift) {
//                currentTask = null;
//            }
            robotHardware.reverseIntake();
            aPressed = true;
        }
        if (!gamepad1.a && aPressed){
            robotHardware.stopIntake();
            aPressed = false;
        }

//        else if (currentTask != intakeAndLift) {
//            robotHardware.stopIntake();
//        }
    }

    /**
     * Define combo task for driver op mode
     */
    void setupCombos() {
        //commented out because when we intake in lift it works for the shared
        //hub but when we do alliance we need to go over the ramps
        //and if the lift is up and the block is in the box while
        //we go over the ramp the block just falls out
//        intakeAndLift = new SequentialComboTask();
//        intakeAndLift.addTask(new AutoIntakeTask(robotHardware));
//        intakeAndLift.addTask(new RobotSleep(500));
//        intakeAndLift.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.MIDDLE));

//        sharedHubTask = new SequentialComboTask();
//        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(25, 25, 10.25);
//        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(100, 100, 10.25);
//        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(10, 10, 10.25);
//        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((15));
//        SampleMecanumDrive drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();
//
//        Pose2d startPos = robotProfile.getProfilePose(startPosStr + "_SHARE_START");
//        Pose2d intakePos_0 = robotProfile.getProfilePose(startPosStr + "_SHARE_INTAKE_0");
//        Pose2d intakePos_1 = robotProfile.getProfilePose(startPosStr + "_SHARE_INTAKE_1");
//        Pose2d intakePos_2 = robotProfile.getProfilePose(startPosStr + "_SHARE_INTAKE_2");
//        Pose2d intakePos_3 = robotProfile.getProfilePose(startPosStr + "_SHARE_INTAKE_3");
//        Pose2d preHubPos_1 = robotProfile.getProfilePose(startPosStr + "_SHARE_PRE_HUB_1");
//        Pose2d hubPos = robotProfile.getProfilePose(startPosStr + "_SHARE_HUB");
//        Pose2d afterHubPos_0 = robotProfile.getProfilePose(startPosStr + "_SHARE_AFTER_HUB");
//
//        Trajectory traj1a = drive.trajectoryBuilder(startPos)
//                .splineToLinearHeading(intakePos_0, intakePos_0.getHeading(), velConstraints, accConstraint)
//                .build();
//        ParallelComboTask par1a = new ParallelComboTask();
//        par1a.addTask(new SplineMoveTask(drive, traj1a));
//        par1a.addTask(new AutoIntakeTask(robotHardware, robotProfile, 5000));
//        sharedHubTask.addTask(par1a);
//
//        Trajectory traj1b = drive.trajectoryBuilder(intakePos_0, true)
//                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading() + Math.PI, velConstraints, accConstraint)
//                .build();
//        sharedHubTask.addTask(new SplineMoveTask(drive, traj1b));
//        sharedHubTask.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.BOTTOM));
//
//        Trajectory traj1c = drive.trajectoryBuilder(preHubPos_1, true)
//                .splineToLinearHeading(hubPos, hubPos.getHeading()+Math.PI,  velConstraints, accConstraint)
//                .build();
//        sharedHubTask.addTask(new SplineMoveTask(drive, traj1c));
//
//        sharedHubTask.addTask(new DeliverToHubTask(robotHardware, robotProfile));
//
//        ParallelComboTask par1b = new ParallelComboTask();
//        Trajectory traj1d = drive.trajectoryBuilder(hubPos)
//                .splineToLinearHeading(afterHubPos_0, afterHubPos_0.getHeading(),  velConstraints, accConstraint)
//                .build();
//        par1b.addTask(new SplineMoveTask(drive, traj1d));
//        sharedHubTask.addTask(par1b);
//
//        Trajectory traj1e = drive.trajectoryBuilder(afterHubPos_0)
//                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading())
//                .build();
//        sharedHubTask.addTask(new SplineMoveTask(drive, traj1e));
//
////        2nd pickup
//        ParallelComboTask par2a = new ParallelComboTask();
//        Trajectory traj2a = drive.trajectoryBuilder(preHubPos_1)
//                .splineToLinearHeading(intakePos_1, intakePos_1.getHeading())
//                .build();
//        par2a.addTask(new SplineMoveTask(drive, traj2a));
//        par2a.addTask(new AutoIntakeTask(robotHardware, robotProfile, 5000));
//        sharedHubTask.addTask(par2a);
//
//        Trajectory traj2b = drive.trajectoryBuilder(intakePos_1, true)
//                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading() + Math.PI, velConstraints, accConstraint)
//                .build();
//        sharedHubTask.addTask(new SplineMoveTask(drive, traj2b));
//        sharedHubTask.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.BOTTOM));
//
//        Trajectory traj2c = drive.trajectoryBuilder(preHubPos_1, true)
//                .splineToLinearHeading(hubPos, hubPos.getHeading()+Math.PI,  velConstraints, accConstraint)
//                .build();
//        sharedHubTask.addTask(new SplineMoveTask(drive, traj2c));
//
//        sharedHubTask.addTask(new DeliverToHubTask(robotHardware, robotProfile));
//
//        ParallelComboTask par2b = new ParallelComboTask();
//        Trajectory traj2d = drive.trajectoryBuilder(hubPos)
//                .splineToLinearHeading(afterHubPos_0, afterHubPos_0.getHeading(),  velConstraints, accConstraint)
//                .build();
//        par2b.addTask(new SplineMoveTask(drive, traj2d));
//        sharedHubTask.addTask(par2b);
//
//        Trajectory traj2e = drive.trajectoryBuilder(afterHubPos_0, true)
//                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading())
//                .build();
//        sharedHubTask.addTask(new SplineMoveTask(drive, traj2e));
//
//        //3rd pick up
//        Trajectory traj3a = drive.trajectoryBuilder(preHubPos_1)
//                .splineToLinearHeading(intakePos_2, intakePos_2.getHeading())
//                .build();
//        ParallelComboTask par3a = new ParallelComboTask();
//        par3a.addTask(new SplineMoveTask(drive, traj3a));
//        par3a.addTask(new AutoIntakeTask(robotHardware, robotProfile, 5000));
//        sharedHubTask.addTask(par3a);
//
//        Trajectory traj3b = drive.trajectoryBuilder(intakePos_2, true)
//                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading() + Math.PI, velConstraints, accConstraint)
//                .build();
//        sharedHubTask.addTask(new SplineMoveTask(drive, traj3b));
//        sharedHubTask.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.BOTTOM));
//
//        Trajectory traj3c = drive.trajectoryBuilder(preHubPos_1, true)
//                .splineToLinearHeading(hubPos, hubPos.getHeading()+Math.PI,  velConstraints, accConstraint)
//                .build();
//        sharedHubTask.addTask(new SplineMoveTask(drive, traj3c));
//
//        sharedHubTask.addTask(new DeliverToHubTask(robotHardware, robotProfile));
//
//        ParallelComboTask par3b = new ParallelComboTask();
//        Trajectory traj3d = drive.trajectoryBuilder(hubPos)
//                .splineToLinearHeading(afterHubPos_0, afterHubPos_0.getHeading(),  velConstraints, accConstraint)
//                .build();
//        par3b.addTask(new SplineMoveTask(drive, traj3d));
//        sharedHubTask.addTask(par3b);
//
//        Trajectory traj3e = drive.trajectoryBuilder(afterHubPos_0, true)
//                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading())
//                .build();
//        sharedHubTask.addTask(new SplineMoveTask(drive, traj3e));
//
//        //4th pick up
//        ParallelComboTask par4a = new ParallelComboTask();
//        Trajectory traj4a = drive.trajectoryBuilder(preHubPos_1)
//                .splineToLinearHeading(intakePos_3, intakePos_3.getHeading())
//                .build();
//        par4a.addTask(new SplineMoveTask(drive, traj4a));
//        par4a.addTask(new AutoIntakeTask(robotHardware, robotProfile, 5000));
//        sharedHubTask.addTask(par4a);
//
//        Trajectory traj4b = drive.trajectoryBuilder(intakePos_3, true)
//                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading() + Math.PI, velConstraints, accConstraint)
//                .build();
//        sharedHubTask.addTask(new SplineMoveTask(drive, traj4b));
//        sharedHubTask.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.MIDDLE));
//
//        Trajectory traj4c = drive.trajectoryBuilder(preHubPos_1, true)
//                .splineToLinearHeading(hubPos, hubPos.getHeading()+Math.PI,  velConstraints, accConstraint)
//                .build();
//        sharedHubTask.addTask(new SplineMoveTask(drive, traj4c));
//
//        sharedHubTask.addTask(new DeliverToHubTask(robotHardware, robotProfile));
//
//        ParallelComboTask par4b = new ParallelComboTask();
//        Trajectory traj4d = drive.trajectoryBuilder(hubPos)
//                .splineToLinearHeading(afterHubPos_0, afterHubPos_0.getHeading(),  velConstraints, accConstraint)
//                .build();
//        par4b.addTask(new SplineMoveTask(drive, traj4d));
//        sharedHubTask.addTask(par4b);
//
//        Trajectory traj4e = drive.trajectoryBuilder(afterHubPos_0)
//                .splineToLinearHeading(preHubPos_1, preHubPos_1.getHeading())
//                .build();
//        sharedHubTask.addTask(new SplineMoveTask(drive, traj4e));
//
//        Trajectory traj4f = drive.trajectoryBuilder(preHubPos_1)
//                .splineToLinearHeading(startPos, startPos.getHeading(),  velConstraints, accConstraint)
//                .build();
//        sharedHubTask.addTask(new SplineMoveTask(drive, traj4f));


        deliverTask = new SequentialComboTask();
        deliverTask.addTask(new DeliverToHubTask(robotHardware, robotProfile));
        deliverTask.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ONE));
    }
}

package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.dashboard.canvas.Spline;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;


@TeleOp(name="Newton DriverOpMode", group="Main")
public class DriverOpMode extends OpMode {
    RobotHardware robotHardware;
    RobotProfile robotProfile;

    enum ActionMode {INTAKE, SHOOTING, REVERSE, STOP};
    ActionMode currentMode = ActionMode.STOP;

    Pose2d currPose;
    Pose2d shootingPose;
    double fieldHeadingOffset;

    boolean fieldMode;
    int fieldModeSign = 1;  // RED side = 1, BLUE side = -1
    boolean dpadRightPressed = false;
    boolean dpadLeftPressed = false;
    boolean aPressed = false;
    boolean yPressed = false;
    boolean xPressed = false;
    boolean dpadUpPressed = false;
    boolean dpadDownPressed = false;

    // DriveThru combos
    SequentialComboTask grabLift, dropWobble;
    SequentialComboTask powerBar;
    SequentialComboTask autoDriveShoot;
    RobotControl currentTask = null;
    RobotVision robotVision;

    @Override
    public void init() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        }
        catch (Exception e) {
        }

        fieldMode = true; //robot starts in field orientation

        Logger.init();
        //Obtain the RobotHardware object from factory
        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        robotHardware.initRobotVision();
        robotVision = robotHardware.getRobotVision();
        robotVision.activateNavigationTarget();
        robotHardware.initLeds();   // need to init everytime

        // Based on the Autonomous mode starting position, define the heading offset for field mode
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        if (prefs.getString(START_POS_MODES_PREF, "RED_1").startsWith("RED")) {
            fieldModeSign = 1;
        }
        else {
            fieldModeSign = -1;
        }

        shootingPose = robotProfile.getProfilePose("SHOOT-DRIVER");
        setupCombos();

    }

    /**
     * handleLED sets LED lights based on
     * Red(Led1) is when it is on robot oriented mode or when we are in an automated task
     * Green(Led2) is when the tracking target is visible
     * Blue(Led3) is when the shooting motor is within the shooting velocity range
     */
    private void handleLED(){
        robotHardware.setLed1(currentTask != null || !fieldMode);
        robotHardware.setLed2(robotVision.isTargetVisible());
        robotHardware.setLed3(robotHardware.isShootingSpeedWithinRange());
    }

    /**
     * Resets the current localization position based on the tracking target by pressing Y
     * and drives to optimal shooting location by pressing left bumper and Y
     */
    private void handleVision() {
        if (!yPressed && gamepad1.y) {
            Pose2d currentPosition = robotVision.getNavigationLocalization();
            if (currentPosition != null) {
                robotHardware.getTrackingWheelLocalizer().setPoseEstimate(currentPosition);
                Logger.logFile("PoseEstimate:" + currentPosition);
            }
            else {
                currentPosition = robotHardware.getTrackingWheelLocalizer().getPoseEstimate();
            }
            if (gamepad1.left_bumper) {
                // determine if we want to go backward or foward based on currPose
                // calculate the angle from current position to the shooting position
                robotHardware.stopIntake();
                robotHardware.startShootMotor();
                robotHardware.ringHolderUp();
                robotHardware.setShooterPosition(true);
                currentMode = ActionMode.SHOOTING;
                double ang = Math.atan2(shootingPose.getX() - currPose.getX(), shootingPose.getY() - currPose.getY());
                double dist = Math.hypot(shootingPose.getX() - currPose.getX(), shootingPose.getY() - currPose.getY());

                if (dist>30 || Math.abs(currPose.getHeading() - shootingPose.getHeading()) < Math.PI / 4) {
                    // use spline move if distance to shooting location is greater than 30 inch away or when we are using the tracking target in the front
                    boolean forward = Math.abs(currPose.getHeading() - ang) < Math.PI / 2;
                    Logger.logFile("Spline From " + currPose + " F:" + forward);
                    Logger.flushToFile();
                    DriveConstraints constraints = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
                    Trajectory traj = robotHardware.getMecanumDrive().trajectoryBuilder(currPose, !forward)
                            .splineToSplineHeading(shootingPose, shootingPose.getHeading(), constraints).build();
                    currentTask = new SplineMoveTask(robotHardware.getMecanumDrive(), traj);
                }
                else {
                    //When is looking at the side target, use the straight line trajectory
                    Logger.logFile("Line From " + currPose);
                    Logger.flushToFile();
                    DriveConstraints constraints = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);

                    Trajectory traj = robotHardware.getMecanumDrive().trajectoryBuilder(currPose, constraints)
                            .lineToLinearHeading(shootingPose).build();
                    currentTask = new SplineMoveTask(robotHardware.getMecanumDrive(), traj);
                }
                currentTask.prepare();
            }
        }
        yPressed = gamepad1.y;
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
        robotHardware.getTrackingWheelLocalizer().update();
        currPose = robotHardware.getTrackingWheelLocalizer().getPoseEstimate();

        handleLED();
        //Handling autonomous task loop
        if (currentTask != null) {
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                currentTask.cleanUp();
                currentTask = null;
            }
            else {
                currentTask.execute();
                if (currentTask.isDone()) {
                    currentTask.cleanUp();
                    Logger.logFile("TaskComplete: " + currentTask + " Pose:" + robotHardware.getTrackingWheelLocalizer().getPoseEstimate());
                    currentTask = null;
                }
            }
        }
        else {
            handleMovement();
            handlePowerBar();
            handleVision();
            handleIntakeAndShoot();
            handleArmAndGrabber();
        }
        telemetry.addData("CurrPose", currPose);
    }

    @Override
    public void stop() {
        try {
            Logger.logFile("DriverOpMode stop() called");
            robotVision.deactivateNavigationTarget();
            Logger.flushToFile();
        }
        catch (Exception e) {
        }
    }

    /**
     * Retrieve set pose from profile
     * @param name name of the robot position
     * @return
     */
    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }

    /**
     * Gamepad Y and right bumper to trigger the endgame sequence
     */
    private void handlePowerBar(){
        if(!yPressed && gamepad1.y) {
            if (gamepad1.right_bumper) {
                Logger.logFile(("pose y " + robotHardware.getMecanumDrive().getPoseEstimate().getY()));
                DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
                DriveConstraints constraints = new DriveConstraints(20.0, 10.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
                powerBar = new SequentialComboTask();
                Pose2d p0 = null, p1 = null;

                if(robotHardware.getMecanumDrive().getPoseEstimate().getY() < -25) {
                    robotHardware.getMecanumDrive().setPoseEstimate(getProfilePose("SHOOT-START-1"));
                     p0 = getProfilePose("SHOOT-START-1");
                     p1 = getProfilePose("SHOOT-LEFT");
                } else {
                    robotHardware.getMecanumDrive().setPoseEstimate(getProfilePose("SHOOT-START-2"));
                    p0 = getProfilePose("SHOOT-START-2");
                    p1 = getProfilePose("SHOOT-RIGHT");
                }
                Trajectory traj1 = robotHardware.getMecanumDrive().trajectoryBuilder(p0, constraints)
                        .lineToLinearHeading(p1, constraints)
                        .build();
                powerBar.addTask(new SplineMoveTask(robotHardware.getMecanumDrive(), traj1));
                Pose2d p2 = getProfilePose("SHOOT-POWER-BAR-3");
                Trajectory traj2 = robotHardware.getMecanumDrive().trajectoryBuilder(p1, moveFast)
                        .splineToSplineHeading(p2, p2.getHeading())
                        .build();
                powerBar.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 1));
                powerBar.addTask(new IntakeMotorTask(robotHardware, robotProfile,  IntakeMotorTask.IntakeMode.STOP));
                powerBar.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
                powerBar.addTask(new ShooterMotorTask(robotHardware, robotProfile, true, robotProfile.hardwareSpec.shootBarVelocity));
                powerBar.addTask(new SplineMoveTask(robotHardware.getMecanumDrive(), traj2));
                powerBar.addTask(new ShootOneRingTask(robotHardware, robotProfile));

                Pose2d p3 = getProfilePose("SHOOT-POWER-BAR-2");
                Trajectory traj3 = robotHardware.getMecanumDrive().trajectoryBuilder(p2, constraints)
                        .lineToLinearHeading(p3, constraints)
                        .build();
                powerBar.addTask(new SplineMoveTask(robotHardware.getMecanumDrive(), traj3));
                powerBar.addTask(new ShootOneRingTask(robotHardware, robotProfile));

                Pose2d p4 = getProfilePose("SHOOT-POWER-BAR-1");
                Trajectory traj4 = robotHardware.getMecanumDrive().trajectoryBuilder(p3, constraints)
                        .lineToLinearHeading(p4, constraints)
                        .build();
                powerBar.addTask(new SplineMoveTask(robotHardware.getMecanumDrive(), traj4));
                powerBar.addTask(new ShootOneRingTask(robotHardware, robotProfile));
                currentTask = powerBar;
                currentTask.prepare();
                currentMode = ActionMode.SHOOTING;
            }
        }
    }

    /**
     * Joystick Driving Controls
     * Left bumper for slow motion
     * Left trigger to enable field mode, right trigger to enable robot-oriented mode
     */
    private void handleMovement() {
        double turn = gamepad1.right_stick_x/2;
        double power = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double moveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI/4;

        if (fieldMode) {
            moveAngle += -currPose.getHeading() - fieldHeadingOffset + fieldModeSign*Math.PI/2;
        }

        //power = power;
        turn = turn / 4;
        if (gamepad1.left_bumper) { // further bring it down
            power = power/3;
            turn = turn/3;
        }
        robotHardware.mecanumDriveTest(power, moveAngle, turn, 0);

        // toggle field mode on/off.
        // Driver 1: left trigger - enable; right trigger - disable
        if (gamepad1.left_trigger > 0) {
            fieldMode = true;
            fieldHeadingOffset = currPose.getHeading();
        } else if (gamepad1.right_trigger > 0) {
            fieldMode = false;  //good luck driving
        }
    }

    /**
     * Uses up  down keypad to switch to different modes between shooting, stop, reverse, and intake
     */
    private void handleIntakeAndShoot() {
        if (!dpadUpPressed && gamepad1.dpad_up) {
            switch (currentMode) {
                case STOP:
                case SHOOTING:
                case REVERSE:
                    robotHardware.startIntake();
                    robotHardware.ringHolderDown();
                    robotHardware.setShooterPosition(true);
                    currentMode = ActionMode.INTAKE;
                    break;
                case INTAKE:
                    robotHardware.reverseIntake();
                    robotHardware.startShootMotor((gamepad1.right_bumper)?robotProfile.hardwareSpec.shootBarVelocity:robotProfile.hardwareSpec.shootVelocity);
                    robotHardware.ringHolderUp();
                    robotHardware.setShooterPosition(true);
                    currentMode = ActionMode.SHOOTING;
                    break;
            }
        }
        dpadUpPressed = gamepad1.dpad_up;
        if (!dpadDownPressed && gamepad1.dpad_down) {
            switch (currentMode) {
                case SHOOTING:
                case REVERSE:
                    robotHardware.stopIntake();
                    robotHardware.stopShootMotor();
                    robotHardware.ringHolderDown();
                    robotHardware.setShooterPosition(true);
                    currentMode = ActionMode.STOP;
                    break;
                case INTAKE:
                case STOP:
                    robotHardware.reverseIntake();
                    robotHardware.stopShootMotor();
                    robotHardware.ringHolderDown();
                    robotHardware.setShooterPosition(true);
                    currentMode = ActionMode.REVERSE;
                    break;
            }
        }
        dpadDownPressed = gamepad1.dpad_down;
        if (gamepad1.x && gamepad1.left_bumper) {  // Auto Drive & Shoot
            currentTask = autoDriveShoot;
            currentTask.prepare();
        }
        else if (gamepad1.x && currentMode==ActionMode.SHOOTING) {
            currentTask = new ShootOneRingTask(robotHardware, robotProfile);
            currentTask.prepare();
        }
    }

    /**
     * Wobble goal pick up and drop
     * Left right dpad to move arm back and forward
     * A key to trigger the pick up sequence
     * B key to trigger the drop sequence
     */
    private void handleArmAndGrabber() {
        if (gamepad1.dpad_right && !dpadRightPressed) {
            robotHardware.setArmNextPosition();
            dpadRightPressed = true;
        }
        dpadRightPressed = gamepad1.dpad_right;
        if (gamepad1.dpad_left && !dpadLeftPressed) {
            robotHardware.setArmPrevPosition();
            dpadLeftPressed = true;
        }
        dpadLeftPressed = gamepad1.dpad_left;
        if (gamepad1.b) {
            currentTask = dropWobble;
            currentTask.prepare();
        }
        if (!aPressed && gamepad1.a) {
            //robotHardware.setGrabberPosition(true);
            currentTask = grabLift;
            grabLift.prepare();
        }
        aPressed = gamepad1.a;
    }

    /**
     * Define combo task for driver op mode
     */
    void setupCombos() {
        grabLift = new SequentialComboTask();
        grabLift.addTask(new GrabberTask(robotHardware, robotProfile, false, 500));
        grabLift.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 300));
        dropWobble = new SequentialComboTask();
        dropWobble.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 300));
        dropWobble.addTask(new GrabberTask(robotHardware, robotProfile, true, 300));
        dropWobble.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 10));
        //Full auto drive sequence
        SequentialComboTask oneAds = new SequentialComboTask();
        oneAds.addTask(new AutoDriveShootTask(robotHardware, robotProfile, AutoDriveShootTask.TaskMode.FIRST_PIC, false));
        oneAds.addTask(new MecanumRotateTask(robotHardware.getMecanumDrive(), Math.PI/4));
        oneAds.addTask(new RobotSleep(100));
        oneAds.addTask(new AutoDriveShootTask(robotHardware, robotProfile, AutoDriveShootTask.TaskMode.MORE_PIC, false));
        oneAds.addTask(new MecanumRotateTask(robotHardware.getMecanumDrive(), Math.PI/4));
        oneAds.addTask(new RobotSleep(100));
        oneAds.addTask(new AutoDriveShootTask(robotHardware, robotProfile, AutoDriveShootTask.TaskMode.MORE_PIC, false));
        oneAds.addTask(new MecanumRotateTask(robotHardware.getMecanumDrive(), Math.PI/4));
        oneAds.addTask(new RobotSleep(100));
        oneAds.addTask(new AutoDriveShootTask(robotHardware, robotProfile, AutoDriveShootTask.TaskMode.MORE_PIC, false));
        oneAds.addTask(new MecanumRotateTask(robotHardware.getMecanumDrive(), Math.PI/4));
        oneAds.addTask(new RobotSleep(100));
        oneAds.addTask(new AutoDriveShootTask(robotHardware, robotProfile, AutoDriveShootTask.TaskMode.MORE_PIC, false));
        oneAds.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));
        oneAds.addTask(new IntakeMotorTask(robotHardware, robotProfile, IntakeMotorTask.IntakeMode.NORMAL));
        oneAds.addTask(new ShooterMotorTask(robotHardware, robotProfile, true));
        AutoDriveShootTask adst = new AutoDriveShootTask(robotHardware, robotProfile, AutoDriveShootTask.TaskMode.DRIVE);
        adst.setEndPose(robotProfile.getProfilePose("AUTO-TRACKER-IMG"));
        oneAds.addTask(adst);
        oneAds.addTask(new RobotSleep(800));
        oneAds.addTask(new VuforiaPoseUpdateTask(robotHardware));
        oneAds.addTask(new SplineMoveTask(robotHardware.getMecanumDrive(), robotProfile.getProfilePose("SHOOT-DRIVER")));
        oneAds.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        oneAds.addTask(new ShootOneRingTask(robotHardware, robotProfile));
        oneAds.addTask(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        oneAds.addTask(new ShootOneRingTask(robotHardware, robotProfile));
        oneAds.addTask(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        oneAds.addTask(new ShootOneRingTask(robotHardware, robotProfile));
        oneAds.addTask(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        oneAds.addTask(new ShooterMotorTask(robotHardware, robotProfile, false));
        autoDriveShoot = new SequentialComboTask();
        autoDriveShoot.addTask(oneAds);
        autoDriveShoot.addTask(oneAds);
        autoDriveShoot.addTask(oneAds);
        autoDriveShoot.addTask(oneAds);
        autoDriveShoot.addTask(oneAds);


//        ArrayList<RobotControl> homePositionList = new ArrayList<RobotControl>();
//        homePositionList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
//                robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra, 100));
//        homePositionList.add(new ClampStraightAngleTask(robotHardware, robotProfile));
//        homePositionList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOrigPos, 100));
//        homePositionList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
//        homePositionList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
//                robotProfile.hardwareSpec.liftHomeReadyPos, 100));
//        homePositionTask = new SequentialComboTask();
//        homePositionTask.setTaskList(homePositionList);
    }


}

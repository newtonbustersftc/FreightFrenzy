package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;


@TeleOp(name="Newton DriverOpMode", group="Main")
public class DriverOpMode extends OpMode {
    RobotHardware robotHardware;
    RobotProfile robotProfile;

    enum ActionMode {INTAKE, SHOOTING, REVERSE, STOP}

    ;
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
    boolean dpadUpPressed = false;
    boolean dpadDownPressed = false;

    // DriveThru combos
    SequentialComboTask grabLift;
    SequentialComboTask powerBar;
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

        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        robotHardware.initRobotVision();
        robotVision = robotHardware.getRobotVision();
        robotVision.activateNavigationTarget();

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

    private void handleVision() {
        if (!yPressed && gamepad1.y) {
            Pose2d currentPosition = robotVision.getNavigationLocalization();
            if (currentPosition != null) {
                robotHardware.getTrackingWheelLocalizer().setPoseEstimate(currentPosition);
                Logger.logFile("PoseEstimate:" + currentPosition);
                if (gamepad1.left_bumper) {
                    // determine if we want to go backward or foward based on currPose
                    // calculate the angle from current position to the shooting position
                    robotHardware.stopIntake();
                    robotHardware.startShootMotor();
                    robotHardware.ringHolderUp();
                    double ang = Math.atan2(shootingPose.getX() - currPose.getX(), shootingPose.getY() - currPose.getY());
                    double dist = Math.hypot(shootingPose.getX() - currPose.getX(), shootingPose.getY() - currPose.getY());
                    if (dist>30 || Math.abs(currPose.getHeading() - shootingPose.getHeading()) < Math.PI / 4) {
                        // use spline move is greater than 20 inch away or the handle is not too much different
                        boolean forward = Math.abs(currPose.getHeading() - ang) < Math.PI / 2;
                        Logger.logFile("Spline From " + currPose + " F:" + forward);
                        Logger.flushToFile();
                        Trajectory traj = robotHardware.getMecanumDrive().trajectoryBuilder(currPose, !forward)
                                .splineToSplineHeading(shootingPose, shootingPose.getHeading()).build();
                        if (currentTask != null) {
                            currentTask.cleanUp();
                        }
                        currentTask = new SplineMoveTask(robotHardware.getMecanumDrive(), traj);
                    }
                    else {
                        Logger.logFile("Line From " + currPose);
                        Logger.flushToFile();
                        DriveConstraints constraints = new DriveConstraints(10.0, 5.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);

                        Trajectory traj = robotHardware.getMecanumDrive().trajectoryBuilder(currPose, constraints)
                                .lineToLinearHeading(shootingPose).build();
                        if (currentTask != null) {
                            currentTask.cleanUp();
                        }
                        currentTask = new SplineMoveTask(robotHardware.getMecanumDrive(), traj);
                    }
                    currentTask.prepare();
                }
            }
        }
        yPressed = gamepad1.y;
    }

    @Override
    public void loop() {
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();
        robotHardware.getTrackingWheelLocalizer().update();
        currPose = robotHardware.getTrackingWheelLocalizer().getPoseEstimate();

        // Robot movement
        handleMovement();
        handlePowerBar();
        handleVision();
        handleIntakeAndShoot();
        handleArmAndGrabber();

        if (currentTask != null) {
            currentTask.execute();
            if (currentTask.isDone()) {
                currentTask.cleanUp();
                Logger.logFile("TaskComplete: " + currentTask + " Pose:" + robotHardware.getTrackingWheelLocalizer().getPoseEstimate());
                currentTask = null;
            }
        }
/*
        if(gamepad1.b && !bAlreadyPressed){
            bAlreadyPressed = true;
            Pose2d shootingPose = new Pose2d(-5, -36, Math.toRadians(0));
            double shootingHeading = 0;
            Trajectory trajectory = robotHardware.getMecanumDrive().trajectoryBuilder(currPose, false).
                    splineToSplineHeading(shootingPose, 0).build();
            currentTask = new SplineMoveTask(robotHardware.getMecanumDrive(), trajectory);
            currentTask.prepare();
        } else if(!gamepad1.b && bAlreadyPressed) {
            bAlreadyPressed = false;
        }
 */
        telemetry.addData("CurrPose", currPose);
        telemetry.addData("Field Mode", fieldMode);
    }

    @Override
    public void stop() {
        // open the clamp to relief the grabber servo
        try {
            Logger.logFile("DriverOpMode stop() called");
            robotVision.deactivateNavigationTarget();
            Logger.flushToFile();
        }
        catch (Exception e) {
        }
    }


    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }

    private void handlePowerBar(){
        if(!yPressed && gamepad1.y) {
                if (gamepad1.right_bumper) {
                    DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
                    DriveConstraints constraints = new DriveConstraints(10.0, 5.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
                    powerBar = new SequentialComboTask();
                    robotHardware.getMecanumDrive().setPoseEstimate(getProfilePose("SHOOT-START"));
                    Pose2d p0 = getProfilePose("SHOOT-START");
                    Pose2d p1 = getProfilePose("SHOOT-POWER-BAR-1");
                    Trajectory traj1 = robotHardware.getMecanumDrive().trajectoryBuilder(p0, moveFast)
                            .splineToSplineHeading(p1, p1.getHeading())
                            .build();
                    powerBar.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
                    powerBar.addTask(new ShooterMotorTask(robotHardware, robotProfile, true, robotProfile.hardwareSpec.shootBarVelocity));
                    powerBar.addTask(new SplineMoveTask(robotHardware.getMecanumDrive(), traj1));
                    powerBar.addTask(new ShootOneRingTask(robotHardware, robotProfile));
                    powerBar.addTask(new RobotSleep(robotProfile.hardwareSpec.shootDelay));

                    Pose2d p2 = getProfilePose("SHOOT-POWER-BAR-2");
                    Trajectory traj2 = robotHardware.getMecanumDrive().trajectoryBuilder(p1, constraints)
                            .lineToLinearHeading(p2, constraints)
                            .build();
                    powerBar.addTask(new SplineMoveTask(robotHardware.getMecanumDrive(), traj2));
                    powerBar.addTask(new ShootOneRingTask(robotHardware, robotProfile));
                    powerBar.addTask(new RobotSleep(robotProfile.hardwareSpec.shootDelay));

                    Pose2d p3 = getProfilePose("SHOOT-POWER-BAR-3");
                    Trajectory traj3 = robotHardware.getMecanumDrive().trajectoryBuilder(p2, constraints)
                            .lineToLinearHeading(p3, constraints)
                            .build();
                    powerBar.addTask(new SplineMoveTask(robotHardware.getMecanumDrive(), traj3));
                    powerBar.addTask(new ShootOneRingTask(robotHardware, robotProfile));
                    powerBar.addTask(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
                    currentTask = powerBar;
                    currentTask.prepare();
                }
            }
        }


    private void handleMovement() {
        if(currentTask!=null){
            return;
        }
        double turn = gamepad1.right_stick_x/2;
        double power = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double moveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI/4;

        if (fieldMode) {
            moveAngle += -currPose.getHeading() - fieldHeadingOffset + fieldModeSign*Math.PI/2;
        }

        power = power / 1.5;
        turn = turn / 4;
        if (gamepad1.left_bumper) { // further bring it down
            power = power/2;
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

    private void handleIntakeAndShoot() {
        if(currentTask!=null){
            return;
        }
        if (!dpadUpPressed && gamepad1.dpad_up) {
            switch (currentMode) {
                case STOP:
                case SHOOTING:
                case REVERSE:
                    robotHardware.startIntake();
                    robotHardware.stopShootMotor();
                    robotHardware.ringHolderDown();
                    robotHardware.setShooterPosition(true);
                    currentMode = ActionMode.INTAKE;
                    break;
                case INTAKE:
                    robotHardware.stopIntake();
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
        if (gamepad1.x) {
            robotHardware.setShooterPosition(false);
        }
        else {
            robotHardware.setShooterPosition(true);
        }
    }

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
            robotHardware.setGrabberPosition(true);
        }
        if (!aPressed && gamepad1.a) {
            //robotHardware.setGrabberPosition(true);
            currentTask = grabLift;
            grabLift.prepare();
        }
        aPressed = gamepad1.a;
    }

    void setupCombos() {
        grabLift = new SequentialComboTask();
        grabLift.addTask(new GrabberTask(robotHardware, robotProfile, false, 500));
        grabLift.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 500));
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

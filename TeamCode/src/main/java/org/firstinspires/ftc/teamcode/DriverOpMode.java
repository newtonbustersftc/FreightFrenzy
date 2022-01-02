package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;


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
    SequentialComboTask intakeAndLift, deliverTask;
    RobotControl currentTask = null;


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
        robotHardware.initRobotVision();
        //robotVision = robotHardware.getRobotVision();
        //robotVision.activateNavigationTarget();
        //robotHardware.initLeds();   // need to init everytime
       // robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        //ensure lift is reset at the beginning and the end
        robotHardware.getRobotVision().initRearCamera();
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

        setupCombos();
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
    }

    /**
     * Uses up  down keypad to switch to different modes between shooting, stop, reverse, and intake
     */
    private void handleIntake() {
        if (gamepad1.right_trigger > 0 && !rightTriggerPressed) {
//            currentTask = intakeAndLift;
//            currentTask.prepare();
            robotHardware.startIntake();
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

        deliverTask = new SequentialComboTask();
        deliverTask.addTask(new DeliverToHubTask(robotHardware, robotProfile));
        deliverTask.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ZERO));
    }
}

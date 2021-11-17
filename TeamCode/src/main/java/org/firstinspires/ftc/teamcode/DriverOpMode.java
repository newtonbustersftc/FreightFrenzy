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

    enum IntakeMode { INTAKE, REVERSE, STOP}
    IntakeMode currIntakeMode = IntakeMode.STOP;

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
    SequentialComboTask deliverTask;

    // DriveThru combos
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
        Logger.logFile("DriverOpMode: " + prefs.getString(START_POS_MODES_PREF, "NONE"));

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
        robotHardware.getLocalizer().update();
        currPose = robotHardware.getLocalizer().getPoseEstimate();

        //currPose = new Pose2d(0,0,0);   // for now
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
                    Logger.logFile("TaskComplete: " + currentTask + " Pose:" + robotHardware.getLocalizer().getPoseEstimate());
                    currentTask = null;
                }
            }
        }

        handleMovement();

        telemetry.addData("CurrPose", currPose);
        telemetry.addData("fieldMode", fieldMode);

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

        if(gamepad1.b){
            robotHardware.setLiftPosition(RobotHardware.LiftPosition.NOT_INIT);
            currentTask = new ResetLiftPositionDriverOpModeTask(robotHardware);
        }

        if(gamepad1.share){
            robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
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
     * Retrieve set pose from profile
     * @param name name of the robot position
     * @return
     */
    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
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
        if (fieldMode) {
            movAngle = padAngle + ((isRedTeam) ? Math.PI / 2 : -Math.PI / 2) - currPose.getHeading();
        } else {
            movAngle = padAngle;
        }
        if (gamepad1.dpad_left) {
            power = power / 3;
            turn = turn / 3;
        }
        robotHardware.mecanumDrive2(power, movAngle, turn);

        // toggle field mode on/off.
        // Driver 1: left trigger - enable; right trigger - disable
        if (gamepad1.dpad_down) {
            fieldMode = true;
        } else if (gamepad1.dpad_right) {
            fieldMode = false;  //good luck driving
        }
    }

    /**
     * Uses up  down keypad to switch to different modes between shooting, stop, reverse, and intake
     */
    private void handleIntake() {
        if (gamepad1.left_trigger > 0) {
            robotHardware.startIntake();
        }
        else if (gamepad1.right_trigger > 0) {
            robotHardware.reverseIntake();
        }
        else {
            robotHardware.stopIntake();
        }



//        if(gamepad1.right_bumper){
//            robotHardware.stopIntake();
//        }

//        dpadLeftPressed = gamepad1.dpad_left;
    }




    /**
     * Define combo task for driver op mode
     */
    void setupCombos() {
        deliverTask = new SequentialComboTask();
        deliverTask.addTask(new DeliverToHubTask(robotHardware, robotProfile));
        deliverTask.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ZERO));
    }
}

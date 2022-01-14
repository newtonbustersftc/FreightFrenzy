package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.GoalTargetRecognition;
import org.firstinspires.ftc.teamcode.util.HubVisionMathModel;

import java.io.File;
import java.util.Locale;

@TeleOp(name="DriverOpMode Test", group="Test")
//@Disabled
public class DriverOpModeTest extends OpMode {
    RobotHardware robotHardware;
    RobotVision robotVision;
    boolean fieldMode;
    boolean isRedTeam;
    RobotProfile robotProfile;

    Pose2d currPose;
    double fieldHeadingOffset;
    boolean upPressed = false;
    boolean downPressed = false;
    double flapServoPos = 0.3;
    double lidServoPos = 0.45;
    boolean rightPressed = false;
    boolean leftPressed = false;
    boolean yPressed = false;
    boolean xPressed = false;
    boolean aPressed = false;
    boolean bPressed = false;
    RobotControl currentTask = null;
    GoalTargetRecognition goalRecog = null;

    @Override
    public void init() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
            Logger.logFile("Exception " + e);
            e.printStackTrace();
        }
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        String startPosStr = prefs.getString("starting position", "none");
        Logger.logFile("StartPos:" + startPosStr);
        isRedTeam = startPosStr.toUpperCase(Locale.ROOT).startsWith("RED");

        fieldMode = true;
        Logger.init();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        robotVision = robotHardware.getRobotVision();
        //robotVision.activateNavigationTarget();
        robotVision.initRearCamera(isRedTeam);  //boolean isRed
        try {
            Thread.sleep(100);
        }
        catch (Exception e) {
        }

        robotVision.startRearCamera();
        robotHardware.initLeds();   // need to init everytime
        robotHardware.boxFlapServo.setPosition(flapServoPos);

        // This is the transformation between the center of the camera and the center of the robot
        Transform2d cameraToRobot = new Transform2d();
// Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        double encoderMeasurementCovariance = 0.8;
// Set to the starting pose of the robot
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        currentTask = new ResetLiftPositionDriverOpModeTask(robotHardware);
        currentTask.prepare();
    }

    @Override
    public void loop() {
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();
        robotHardware.getLocalizer().update();
        currPose = robotHardware.getLocalizer().getPoseEstimate();

        handleMovement();
        // field mode or not
        if (gamepad1.left_trigger > 0) {
            fieldMode = true;
            fieldHeadingOffset = currPose.getHeading();
        } else if (gamepad1.right_trigger > 0) {
            fieldMode = false;  //good luck driving
        }
        testHardware();
        testHubVision();

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
    }

    @Override
    public void stop() {
        // open the clamp to relief the grabber servo
        try {
            robotVision.stopRearCamera();
            robotHardware.stopAll();
            Logger.logFile("DriverOpMode Test stop() called");
            Logger.flushToFile();
        } catch (Exception e) {
            Log.e("DriverOpMode", Log.getStackTraceString(e));
        }

    }

    private void handleMovement() {
        double turn = gamepad1.right_stick_x/4;
        double power = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double padAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) + Math.PI/2;

        double movAngle;
        if (fieldMode) {
            movAngle = padAngle+((isRedTeam)?Math.PI/2:-Math.PI/2) - currPose.getHeading();
        }
        else {
            movAngle = padAngle;
        }
        if (gamepad1.left_bumper) {
            power = power/3;
            turn = turn/3;
        }
        robotHardware.mecanumDrive2(power, movAngle, turn);
//       telemetry.addData("Stick: ", Math.toDegrees(padAngle));
//       telemetry.addData("Move: ", Math.toDegrees(movAngle));
//       telemetry.addData("Power:", power);
        telemetry.addData("Move",  "a:" + Math.toDegrees(padAngle));
        telemetry.addData("Stick", "x:" + gamepad1.left_stick_x + " y:" + gamepad1.left_stick_y);
        telemetry.addData("Mode: ", (fieldMode)?"Field":"Robot");

       // toggle field mode on/off.
       // Driver 1: left trigger - enable; right trigger - disable
        if (gamepad1.left_trigger > 0) {
            fieldMode = true;
        } else if (gamepad1.right_trigger > 0) {
            fieldMode = false;  //good luck driving
        }
        telemetry.addData("Pose:", robotHardware.getLocalizer().getPoseEstimate());
    }

    private void testHardware() {
        // test lift
        int currLiftPos = robotHardware.getEncoderCounts(RobotHardware.EncoderType.LIFT);
        if (gamepad1.dpad_up && !upPressed) {
            robotHardware.setLiftMotorPosition(currLiftPos+20);
        }
        upPressed = gamepad1.dpad_up;
        if (gamepad1.dpad_down && !downPressed) {
            robotHardware.setLiftMotorPosition(currLiftPos-20);
        }
        downPressed = gamepad1.dpad_down;
        if (gamepad1.dpad_left && !leftPressed) {
            lidServoPos = lidServoPos - 0.02;
            robotHardware.boxLidServo.setPosition(lidServoPos);
        }
        leftPressed = gamepad1.dpad_left;
        if (gamepad1.dpad_right && !rightPressed) {
            lidServoPos = lidServoPos + 0.02;
            robotHardware.boxLidServo.setPosition(lidServoPos);
        }
        rightPressed = gamepad1.dpad_right;
        // test flapServo
        if (gamepad1.x && !xPressed) {
            flapServoPos = flapServoPos - 0.02;
            robotHardware.boxFlapServo.setPosition(flapServoPos);
        }
        xPressed = gamepad1.x;
        if (gamepad1.y && !yPressed) {
            flapServoPos = flapServoPos + 0.02;
            robotHardware.boxFlapServo.setPosition(flapServoPos);
        }
        yPressed = gamepad1.y;
        // test localizer reset
        telemetry.addData("Alpha:", robotHardware.getColorSensorAlpha());
        telemetry.addData("Lift:", currLiftPos);
        telemetry.addData("Flap:", flapServoPos);
        telemetry.addData("Lid:", lidServoPos);
    }

    public void testHubVision() {
        robotVision.getLastHubResult();
        HubVisionMathModel.Result r = robotVision.getLastHubResult();
        if (r!=null) {
            telemetry.addData("Hub", r);
            telemetry.addData("Err:", r.getAngleCorrection());
        }
        if (gamepad1.a && !aPressed) {
            robotVision.saveNextImage();
        }
        aPressed = gamepad1.a;
        telemetry.addData("VidTPS", robotVision.getHubVicTps());
        if (gamepad1.b && !bPressed) {
            currentTask = new AutoHubApproachTask(robotHardware, robotProfile);
            currentTask.prepare();
        }
        bPressed = gamepad1.b;
    }
}
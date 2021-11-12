package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;
import org.firstinspires.ftc.teamcode.util.GoalTargetRecognition;

import java.io.File;

@TeleOp(name="DriverOpMode Test", group="Test")
//@Disabled
public class DriverOpModeTest extends OpMode {
    RobotHardware robotHardware;
    RobotVision robotVision;
    boolean fieldMode;
    RobotProfile robotProfile;

    Pose2d currPose;
    double fieldHeadingOffset;
    boolean dpadLeftDown = false;
    boolean dpadRightDown = false;
    double shootServoPos = 0.6;
    boolean yPressed = false;
    boolean aPressed = false;
    boolean bPressed = false;
    GoalTargetRecognition goalRecog = null;

    @Override
    public void init() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        fieldMode = true;
        Logger.init();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        robotHardware.initRobotVision();
        //robotVision = robotHardware.getRobotVision();
        //robotVision.activateNavigationTarget();
        robotHardware.initLeds();   // need to init everytime

        // This is the transformation between the center of the camera and the center of the robot
        Transform2d cameraToRobot = new Transform2d();
// Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        double encoderMeasurementCovariance = 0.8;
// Set to the starting pose of the robot
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
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
        // test controls
        if (gamepad1.dpad_up) {
            robotHardware.startIntake();
        }
        else {
            robotHardware.stopIntake();
        }
        if (gamepad1.dpad_right) {
            robotHardware.startDuck(-1);
        }
        else {
            robotHardware.stopDuck();
        }
        if (gamepad1.x) {
            robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,00,0));
        }
        if (gamepad1.y) {
            robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,Math.PI/2));
        }

        telemetry.addData("Pose:", robotHardware.getLocalizer().getPoseEstimate());
    }

    @Override
    public void stop() {
        // open the clamp to relief the grabber servo
        try {
            robotHardware.stopAll();
            Logger.logFile("DriverOpMode Test stop() called");
            Logger.flushToFile();
        } catch (Exception e) {
            Log.e("DriverOpMode", Log.getStackTraceString(e));
        }
    }
    private void handleMovement() {
        double turn = gamepad1.right_stick_x / 2;
        double power = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double moveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4.5;

        if (fieldMode) {
            moveAngle += currPose.getHeading() - fieldHeadingOffset - Math.PI / 2;
        }

        if (gamepad1.left_bumper) {
            power = power / 3.5;
            turn = turn / 13;
        }
        robotHardware.mecanumDriveTest(power, moveAngle, turn, 0);
    }
}
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
    T265Camera slamra;
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
        //robotHardware.initRobotVision();
        //robotVision = robotHardware.getRobotVision();
        //robotVision.activateNavigationTarget();
        robotHardware.initLeds();   // need to init everytime

        // This is the transformation between the center of the camera and the center of the robot
        Transform2d cameraToRobot = new Transform2d();
// Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        double encoderMeasurementCovariance = 0.8;
// Set to the starting pose of the robot
        com.arcrobotics.ftclib.geometry.Pose2d startingPose = new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d());

        slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        slamra.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin

// Call this when you're ready to get camera updates
        slamra.start();
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
    }

    @Override
    public void loop() {
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();
        robotHardware.getTrackingWheelLocalizer().update();
        currPose = robotHardware.getTrackingWheelLocalizer().getPoseEstimate();

        handleMovement();
        // field mode or not
        if (gamepad1.left_trigger > 0) {
            fieldMode = true;
            fieldHeadingOffset = currPose.getHeading();
        } else if (gamepad1.right_trigger > 0) {
            fieldMode = false;  //good luck driving
        }
        // test controls
        if (gamepad1.dpad_right && !dpadRightDown) {
            robotHardware.setArmNextPosition();
            dpadRightDown = true;
        }
        else {
            dpadRightDown = gamepad1.dpad_right;
        }
        if (gamepad1.dpad_left && !dpadLeftDown) {
            robotHardware.setArmPrevPosition();
            dpadLeftDown = true;
        }
        else {
            dpadLeftDown = gamepad1.dpad_left;
        }

        //robotHardware.setShooterPosition(gamepad1.x);
        if (gamepad1.y && !yPressed) {
            goalRecog = robotHardware.getRobotVision().getGoalTargetRecognition();
        }
        yPressed = gamepad1.y;
        if (gamepad1.a && !aPressed) {
            shootServoPos -= 0.02;
            robotHardware.shootServo.setPosition(shootServoPos);
        }
        aPressed = gamepad1.a;
        if (gamepad1.b && !bPressed) {
            shootServoPos += 0.02;
            robotHardware.shootServo.setPosition(shootServoPos);
        }
        bPressed = gamepad1.b;
        if (gamepad1.dpad_up) {
            robotHardware.startShootMotor();
            robotHardware.ringHolderUp();
        }
        if (gamepad1.dpad_down) {
            robotHardware.stopShootMotor();
            robotHardware.ringHolderDown();
        }
        //telemetry.addData("LeftE", robotHardware.getEncoderCounts(RobotHardware.EncoderType.LEFT));
        //telemetry.addData("RightE", robotHardware.getEncoderCounts(RobotHardware.EncoderType.RIGHT));
        //telemetry.addData("HorizE", robotHardware.getEncoderCounts(RobotHardware.EncoderType.HORIZONTAL));


        T265Camera.CameraUpdate camUpdate = slamra.getLastReceivedCameraUpdate();
        telemetry.addData("Pose:", robotHardware.getTrackingWheelLocalizer().getPoseEstimate());
        telemetry.addData("X:", camUpdate.pose.getX()*100/2.54);
        telemetry.addData("Y:", camUpdate.pose.getY()*100/2.54);
        telemetry.addData("H:", Math.toDegrees(camUpdate.pose.getHeading()));
        //telemetry.addData("Shoot Servo:", shootServoPos);
        //telemetry.addData("ArmPos", robotHardware.getEncoderCounts(RobotHardware.EncoderType.ARM));
        if (goalRecog!=null) {
            telemetry.addData("Goal Dist", goalRecog.getDistanceInch());
            telemetry.addData("Goal Angle", Math.toDegrees(goalRecog.getTargetAngle()));
        }
   }

    @Override
    public void stop() {
        // open the clamp to relief the grabber servo
        try {
            slamra.stop();
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
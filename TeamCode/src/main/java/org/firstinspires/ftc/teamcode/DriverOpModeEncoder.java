package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

@TeleOp(name="DriverOpMode Test", group="Test")
//@Disabled
public class DriverOpModeEncoder extends OpMode {
    RobotHardware robotHardware;
    RobotNavigator navigator;
    boolean fieldMode;
    RobotProfile robotProfile;

    Pose2d currPose;
    double fieldHeadingOffset;
    boolean dpadLeftDown = false;
    boolean dpadRightDown = false;

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
        if (gamepad1.x) {
            robotHardware.setGrabberPosition(false);
        }
        if (gamepad1.y) {
            robotHardware.setGrabberPosition(true);
        }

        telemetry.addData("LeftE", robotHardware.getEncoderCounts(RobotHardware.EncoderType.LEFT));
        telemetry.addData("RightE", robotHardware.getEncoderCounts(RobotHardware.EncoderType.RIGHT));
        telemetry.addData("HorizE", robotHardware.getEncoderCounts(RobotHardware.EncoderType.HORIZONTAL));
        telemetry.addData("Pose:", robotHardware.getTrackingWheelLocalizer().getPoseEstimate());
        //telemetry.addData("Velo:", robotHardware.getTrackingWheelLocalizer().getPoseVelocity());
        telemetry.addData("ArmPos", robotHardware.getEncoderCounts(RobotHardware.EncoderType.ARM));
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
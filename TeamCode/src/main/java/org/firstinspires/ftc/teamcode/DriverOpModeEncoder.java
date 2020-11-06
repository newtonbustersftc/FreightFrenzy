package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

@TeleOp(name="DriverOpMode Encoder", group="Test")
//@Disabled
public class DriverOpModeEncoder extends OpMode {
    RobotHardware robotHardware;
    RobotNavigator navigator;
    boolean fieldMode;
    RobotProfile robotProfile;

//    double forward, strafe, rotation;

    double gyroAngleOffset;
    double gyroCurrAngle;
    int liftEncoderCnt;
    int sliderEncoderCnt;

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

        gyroAngleOffset = robotHardware.getGyroAngle();

        navigator = new RobotNavigator(robotProfile);
        navigator.reset();
        navigator.setInitPosition(0, 0, 0);
//        if(!robotHardware.imu1.isGyroCalibrated()){
//
//        }
    }

    @Override
    public void loop() {
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();
        robotHardware.getTrackingWheelLocalizer().update();
        gyroCurrAngle = robotHardware.getGyroAngle();

        navigator.updateEncoderPos(robotHardware.getEncoderCounts(RobotHardware.EncoderType.LEFT),
                robotHardware.getEncoderCounts(RobotHardware.EncoderType.RIGHT),
                robotHardware.getEncoderCounts(RobotHardware.EncoderType.HORIZONTAL));

        // Robot movement
        handleMovement();

        // toggle field mode on/off.
        // Driver 1: left trigger - enable; right trigger - disable
        if (gamepad1.left_trigger>0) {
            fieldMode = true;   // easy mode
            gyroAngleOffset = gyroCurrAngle;
        }
        else if (gamepad1.right_trigger>0) {
            fieldMode = false;  //good luck driving
        }

        telemetry.addData("Gyro", Math.toDegrees(gyroCurrAngle));
        //telemetry.addData("LeftE", robotHardware.getEncoderCounts(RobotHardware.EncoderType.LEFT));
        //telemetry.addData("RightE", robotHardware.getEncoderCounts(RobotHardware.EncoderType.RIGHT));
        //telemetry.addData("HorizE", robotHardware.getEncoderCounts(RobotHardware.EncoderType.HORIZONTAL));
        telemetry.addData("Pose:", robotHardware.getTrackingWheelLocalizer().getPoseEstimate());
        telemetry.addData("Velo:", robotHardware.getTrackingWheelLocalizer().getPoseVelocity());
//        telemetry.addData("X", navigator.getWorldX());
//        telemetry.addData("Y", navigator.getWorldY());
//        telemetry.addData("Angle:", navigator.getHeading());
   }

   private void handleMovement() {
       double power = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
       double moveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI/4;
       if (fieldMode) {
           moveAngle += Math.toRadians(gyroCurrAngle - gyroAngleOffset);
       }
       robotHardware.mecanumDriveTest(power, moveAngle, gamepad1.right_stick_x,0);
   }
}
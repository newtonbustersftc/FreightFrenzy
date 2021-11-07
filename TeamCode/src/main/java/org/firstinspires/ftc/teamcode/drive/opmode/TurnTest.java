package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double imuOffset = drive.getRawExternalHeading();

        waitForStart();
        telemetry.clearAll();
        while (!isStopRequested()) {
            if (gamepad1.a) {
                while (gamepad1.a) {
                    idle();
                }
                drive.turn(Math.toRadians(ANGLE));
            }
            if (gamepad1.b) {
                while (gamepad1.a) {
                    idle();
                }
                drive.turn(Math.toRadians(-ANGLE));
            }
            telemetry.addLine("Use A/B to turn left or right");
            telemetry.addData("Pose2d:", drive.getPoseEstimate());
            telemetry.addData("IMU:", Math.toDegrees(drive.getRawExternalHeading() - imuOffset));
            telemetry.update();
        }
    }
}

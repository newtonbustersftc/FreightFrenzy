package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.clearAll();
        waitForStart();

        if (isStopRequested()) return;
        telemetry.addData("Pose2d:", drive.getPoseEstimate());
        telemetry.update();
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(36, -15), 0)
                .splineTo(new Vector2d(48, -15), 0)
                .splineTo(new Vector2d(84, 6), 0)
                .build();

        drive.followTrajectory(traj);
        telemetry.addData("Pose2d:", drive.getPoseEstimate());
        telemetry.update();

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(48, -15), Math.toRadians(180))
                        .splineTo(new Vector2d(36, -15), Math.toRadians(180))
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
        telemetry.addData("Pose2d:", drive.getPoseEstimate());
        telemetry.update();
        sleep(2000);
    }
}

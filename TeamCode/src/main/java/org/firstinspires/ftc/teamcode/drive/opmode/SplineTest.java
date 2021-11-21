package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logger;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotProfile;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.File;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive;
        RobotProfile robotProfile = null;
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        Logger.init();
        RobotHardware robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        drive = (SampleMecanumDrive)robotHardware.getMecanumDrive();

        telemetry.clearAll();
        waitForStart();
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));

        if (isStopRequested()) return;
        Logger.logFile("Pose2d: "+ drive.getPoseEstimate());
        Logger.flushToFile();
        DriveConstraints constraints = new DriveConstraints(20.0, 15.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(0,0),0)
//               .lineTo(new Vector2d(0, -12.2), constraints)
//                .strafeRight(10)
                .splineTo(new Vector2d(48, 15), 0, constraints)
                .splineTo(new Vector2d(84, -6), 0, constraints)
                .build();

        drive.followTrajectory(traj);
        Logger.logFile("Pose2d:"+ drive.getPoseEstimate());
        Logger.flushToFile();

        sleep(2000);

//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(48, -15), Math.toRadians(180))
//                        .splineTo(new Vector2d(36, -15), Math.toRadians(180))
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
//        telemetry.addData("Pose2d:", drive.getPoseEstimate());
//        telemetry.update();
//        sleep(2000);
    }
}

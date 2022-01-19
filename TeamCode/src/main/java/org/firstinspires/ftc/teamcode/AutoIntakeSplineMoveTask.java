package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutoIntakeSplineMoveTask implements RobotControl {
    RobotHardware hardware;
    SampleMecanumDrive drive;
    Trajectory trajectory;
    RobotHardware.Freight freight;
    RobotProfile robotProfile;
    Pose2d nextPos;

    public AutoIntakeSplineMoveTask(SampleMecanumDrive drive, Trajectory trajectory, RobotHardware hardware, Pose2d nextPos){
        this.drive = drive;
        this.trajectory = trajectory;
        this.hardware = hardware;
        this.nextPos = nextPos;
    }
    @Override
    public void prepare() {
        drive.followTrajectoryAsync(trajectory);
        freight = RobotHardware.Freight.NONE;
    }

    @Override
    public void execute() {
        freight = hardware.getFreight();
        drive.update();
    }

    @Override
    public void cleanUp() {
        Logger.logFile("AutoIntakeSplineMove clean up");
        hardware.setMotorPower(0, 0, 0, 0);

        if(nextPos !=null) {  //only for ParallelComboIntakeMovePriority
            Pose2d curPos = hardware.getLocalizer().getPoseEstimate();
            Logger.logFile("pick up pose in AutoIntakeSplineMoveTask: " + curPos);
            Trajectory traj = drive.trajectoryBuilder(curPos, true)
                    .splineToLinearHeading(nextPos, nextPos.getHeading() + Math.PI)
                    .build();
            drive.followTrajectoryAsync(traj);
        }
    }

    @Override
    public boolean isDone() {
        if(freight != RobotHardware.Freight.NONE){
            Logger.logFile("pick up pose in autoIntakeSplineMove: " + hardware.getLocalizer().getPoseEstimate());
        }
        return !drive.isBusy() || freight != RobotHardware.Freight.NONE;
    }
}

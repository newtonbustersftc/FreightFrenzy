package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutoIntakeSplineMoveTask implements RobotControl {
    RobotHardware hardware;
    SampleMecanumDrive drive;
    Trajectory trajectory;
    RobotHardware.Freight freight;
    Pose2d nextPos;

    public AutoIntakeSplineMoveTask(Trajectory trajectory, RobotHardware hardware, Pose2d nextPos){
        this.drive = hardware.mecanumDrive;
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
        TrajectoryAccelerationConstraint slowAccConstraint = SampleMecanumDrive.getAccelerationConstraint((10));
        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(10, 10, 10.25);

        Logger.logFile("AutoIntakeSplineMove clean up");
        hardware.setMotorPower(0, 0, 0, 0);

        if(nextPos !=null) {  //only for ParallelComboIntakeMovePriority
            Pose2d curPos = hardware.getLocalizer().getPoseEstimate();
            Logger.logFile("pick up pose in AutoIntakeSplineMoveTask: " + curPos);
            Trajectory traj = drive.trajectoryBuilder(curPos, true)
                    .splineToLinearHeading(nextPos, nextPos.getHeading() + Math.PI, slowVelConstraints, slowAccConstraint)
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

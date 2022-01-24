package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class InstantaneousPostionTrajectoryTask implements RobotControl {
    RobotHardware robotHardware;
    SampleMecanumDrive drive;
    Pose2d nextPos;
    boolean isBackward;

    public InstantaneousPostionTrajectoryTask(RobotHardware robotHardware, Pose2d nextPos, boolean isBackward){
        this.robotHardware = robotHardware;
        this.drive = robotHardware.mecanumDrive;
        this.nextPos = nextPos;
        this.isBackward = isBackward;
    }

    @Override
    public void prepare() {
        Trajectory traj = null;
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((40));
        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(25, 25, 10.25);

        Pose2d curPos = robotHardware.getLocalizer().getPoseEstimate();
        Logger.logFile("pick up pose in InstantaneousPositionTrajectoryTask: " + curPos);
        if(isBackward) {
            traj = drive.trajectoryBuilder(curPos, true)
                    .splineTo(nextPos.vec(), nextPos.getHeading() + Math.PI, velConstraints, accConstraint)
                    .build();
        }else{
            traj = drive.trajectoryBuilder(curPos)
                    .splineTo(nextPos.vec(), nextPos.getHeading(), velConstraints, accConstraint)
                    .build();
        }
        drive.followTrajectoryAsync(traj);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void cleanUp() {

    }

    @Override
    public boolean isDone() {
        return !drive.isBusy();
    }
}

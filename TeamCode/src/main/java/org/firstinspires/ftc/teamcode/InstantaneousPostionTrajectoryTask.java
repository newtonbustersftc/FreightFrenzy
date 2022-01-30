package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class InstantaneousPostionTrajectoryTask implements RobotControl {
    RobotHardware robotHardware;
    SampleMecanumDrive drive;
    Pose2d nextPos;
    boolean isBackward;
    ArrayList<Pose2d> poses;
    ArrayList<TrajectoryVelocityConstraint> poseSpeed;

    public InstantaneousPostionTrajectoryTask(RobotHardware robotHardware, Pose2d nextPos, boolean isBackward){
        this.robotHardware = robotHardware;
        this.drive = robotHardware.mecanumDrive;
        this.nextPos = nextPos;
        this.isBackward = isBackward;
    }

    public InstantaneousPostionTrajectoryTask(RobotHardware robotHardware, ArrayList<Pose2d> poses, ArrayList<TrajectoryVelocityConstraint> poseSpeed, boolean isBackward){
        this.robotHardware = robotHardware;
        this.drive = robotHardware.mecanumDrive;
        this.poses = poses;
        this.isBackward = isBackward;
        this.poseSpeed = poseSpeed;
    }



    @Override
    public void prepare() {
        TrajectoryBuilder trajectoryBuilder ;
        Trajectory traj;
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((40));
        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(25, 25, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(50, 50, 10.25);
        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(10, 10, 10.25);

        Pose2d curPos = robotHardware.getLocalizer().getPoseEstimate();
        Logger.logFile("pick up pose in InstantaneousPositionTrajectoryTask: " + curPos);
        if(isBackward) {
            if(poses == null){  //only nextPos to handle
                traj = drive.trajectoryBuilder(curPos, true)
                        .splineTo(nextPos.vec(), nextPos.getHeading() + Math.PI, velConstraints, accConstraint)
                        .build();
            }else {
                trajectoryBuilder = drive.trajectoryBuilder(curPos, true);
                for(int i = 0; i<poses.size(); i++){
                    Pose2d pose = poses.get(i);
                    TrajectoryVelocityConstraint myVelocityConstraint = poseSpeed.get(i);
                    trajectoryBuilder.splineTo(pose.vec(), pose.getHeading()+Math.PI, myVelocityConstraint, accConstraint );
                }
//                Pose2d pose = poses.get(poses.size()-1);
//                trajectoryBuilder.splineToLinearHeading(pose
//                        , pose.getHeading()+Math.PI, velConstraints, accConstraint );
                traj = trajectoryBuilder.build();
            }
        }else{
            if(poses == null){
                traj = drive.trajectoryBuilder(curPos)
                        .splineTo(nextPos.vec(), nextPos.getHeading(), velConstraints, accConstraint)
                        .build();
            }else{
                trajectoryBuilder = drive.trajectoryBuilder(curPos, true);
                for(int i = 0; i<poses.size()-1; i++){
                    Pose2d pose = poses.get(i);
                    trajectoryBuilder.splineTo(pose.vec(), pose.getHeading()+Math.PI, fastVelConstraints, accConstraint );
                }
                Pose2d pose = poses.get(poses.size()-1);
                trajectoryBuilder.splineTo(pose.vec(), pose.getHeading()+Math.PI, velConstraints, accConstraint );
                traj = trajectoryBuilder.build();
            }
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

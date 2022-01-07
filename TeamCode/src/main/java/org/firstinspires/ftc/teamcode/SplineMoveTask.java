package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * SplineMoveTask
 * Created by Gavin Fountain
 */

public class SplineMoveTask implements RobotControl {

    SampleMecanumDrive drive;
    Trajectory trajectory;
    TrajectorySequence trajectorySequence;
    Pose2d targetPose;
    TrajectoryVelocityConstraint velocityConstraint;

    public SplineMoveTask(SampleMecanumDrive drive, Trajectory trajectory){
        this.drive = drive;
        this.trajectory = trajectory;
        targetPose = null;
    }
    public SplineMoveTask(SampleMecanumDrive drive, TrajectorySequence trajectory){
        this.drive = drive;
        this.trajectorySequence = trajectory;
        targetPose = null;
    }

    public SplineMoveTask(SampleMecanumDrive drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;
    }

    public String toString() {
        if(trajectorySequence == null)
            return "SplineMove " + trajectory.start() + " -> " + trajectory.end() ;
        else
            return "SplineMove " + trajectorySequence.start() + " -> " + trajectorySequence.end() ;
    }

    public boolean isDone(){
        return !drive.isBusy();
    }

    public void prepare(){
        if (targetPose!=null) {
            Pose2d currPose = drive.getPoseEstimate();
            double ang = Math.atan2(targetPose.getX() - currPose.getX(), targetPose.getY() - currPose.getY());
            boolean forward = Math.abs(currPose.getHeading() - ang) < Math.PI / 2;
            if (trajectorySequence == null) {
                trajectory = drive.trajectoryBuilder(currPose, !forward)
                        .splineToSplineHeading(targetPose, targetPose.getHeading()).build();
            }else{
                trajectorySequence = drive.trajectorySequenceBuilder(currPose)
                        .splineToSplineHeading(targetPose, targetPose.getHeading()).build();
            }
        }
        if(trajectorySequence == null)
            drive.followTrajectoryAsync(trajectory);
        else
            drive.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void execute() {
        drive.update();
    }

    public void cleanUp(){

    }

}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.BulkMecanumDrive;

/**
 * SplineMoveTask
 * Created by Gavin Fountain
 */

public class SplineMoveTask implements RobotControl {

    BulkMecanumDrive drive;
    Trajectory trajectory;
    Pose2d targetPose;

    public SplineMoveTask(BulkMecanumDrive drive, Trajectory trajectory){
        this.drive = drive;
        this.trajectory = trajectory;
        targetPose = null;
    }

    public SplineMoveTask(BulkMecanumDrive drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;
    }

    public String toString() {
        return "SplineMove " + trajectory.start() + " -> " + trajectory.end();
    }

    public boolean isDone(){
        return !drive.isBusy();
    }

    public void prepare(){
        if (targetPose!=null) {
            Pose2d currPose = drive.getPoseEstimate();
            double ang = Math.atan2(targetPose.getX() - currPose.getX(), targetPose.getY() - currPose.getY());
            boolean forward = Math.abs(currPose.getHeading() - ang) < Math.PI / 2;
            trajectory = drive.trajectoryBuilder(currPose, !forward)
                            .splineToSplineHeading(targetPose, targetPose.getHeading()).build();
        }
        drive.followTrajectoryAsync(trajectory);
    }

    public void execute() {
        drive.update();
    }

    public void cleanUp(){

    }

}

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
        if (targetPose==null) {
            return "SplineMove " + trajectory.start() + " -> " + trajectory.end();
        }
        else {
            return "SplineMove curr ->" + targetPose;
        }
    }

    public boolean isDone(){
        return !drive.isBusy();
    }

    public void prepare(){
        if (targetPose!=null) {
            trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
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

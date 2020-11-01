package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * SplineMoveTask
 * Created by Gavin Fountain
 */

public class SplineMoveTask implements RobotControl {

    SampleMecanumDrive drive;
    Trajectory trajectory;

    public SplineMoveTask(SampleMecanumDrive drive, Trajectory trajectory){
        this.drive = drive;
        this.trajectory = trajectory;
    }

    public String toString() {
        return "SplineMove " + trajectory.start() + " -> " + trajectory.end();
                //+ " curr:" + navigator.getLocationString();
    }

    public boolean isDone(){
        return !drive.isBusy();
    }

    public void prepare(){
        drive.followTrajectory(trajectory);
    }

    public void execute() {
        drive.update();
    }

    public void cleanUp(){

    }

}

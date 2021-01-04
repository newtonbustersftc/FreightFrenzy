package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.BulkMecanumDrive;

/**
 * SplineMoveTask
 * Created by Gavin Fountain
 */

public class MecanumRotateTask implements RobotControl {

    BulkMecanumDrive drive;
    double angle;

    public MecanumRotateTask(BulkMecanumDrive drive, double angle){
        this.drive = drive;
        this.angle = angle;
    }

    public String toString() {
        return "MecanumRotate " + Math.toDegrees(angle);
                //+ " curr:" + navigator.getLocationString();
    }

    public boolean isDone(){
        return !drive.isBusy();
    }

    public void prepare(){
        drive.turnAsync(angle);
    }

    public void execute() {
        drive.update();
    }

    public void cleanUp(){

    }

}

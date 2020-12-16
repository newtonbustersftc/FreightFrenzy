package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

public class WaitForPoseTask implements RobotControl {
    StandardTrackingWheelLocalizer localizer;
    Pose2d pos1, pos2;
    boolean isDone = false;

    public WaitForPoseTask(StandardTrackingWheelLocalizer localizer, Pose2d low, Pose2d high) {
        this.localizer = localizer;
        this.pos1 = low;
        this.pos2 = high;
    }

    public String toString() {
        return "Location Trigger: " + pos1 + " to " + pos2;
    }

    @Override
    public void prepare() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void cleanUp() {

    }

    // isDone only when the navigator x,y,h all within the range of pos1 and pos2
    @Override
    public boolean isDone() {
        double ratioX = 0;
        double ratioY = 0;
        double ratioA = 0;
        Pose2d currPose = localizer.getPoseEstimate();
        if (pos1.getX() != pos2.getX()) {
            ratioX = (currPose.getX() - pos1.getX())/(pos2.getX() - pos1.getX());
        }
        if (pos1.getY() != pos2.getY()) {
            ratioY = (currPose.getY() - pos1.getY())/(pos2.getY() - pos1.getY());
        }
        if (pos1.getHeading() != pos2.getHeading()) {
            ratioA = (currPose.getHeading() - pos1.getHeading())/(pos2.getHeading() - pos1.getHeading());
        }
        return ratioX>0 && ratioX<1.0 && ratioY>0 && ratioY<1.0 && ratioA>0 && ratioA<1.0;
    }
}

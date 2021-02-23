package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.BulkMecanumDrive;

import java.util.ArrayList;
import java.util.List;

public class VuforiaPoseUpdateTask implements RobotControl {
    RobotHardware robotHardware;

    public VuforiaPoseUpdateTask(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    public String toString(){
        return "Vuforia Pose Update Task";
    }

    public void prepare() {
    }

    public void execute() {
        if (robotHardware.getRobotVision().isTargetVisible()) {
            Pose2d pose = robotHardware.getRobotVision().getNavigationLocalization();
            robotHardware.getTrackingWheelLocalizer().setPoseEstimate(pose);
            Logger.logFile("Update PoseEstimate to " + pose);
        }
        else {
            Logger.logFile("Vuforia target not visible");
        }
    }

    public void cleanUp() {
    }

    public boolean isDone() {
        return true;
    }
}

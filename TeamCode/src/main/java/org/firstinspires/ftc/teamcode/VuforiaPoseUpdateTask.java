package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

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
            robotHardware.getLocalizer().setPoseEstimate(pose);
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

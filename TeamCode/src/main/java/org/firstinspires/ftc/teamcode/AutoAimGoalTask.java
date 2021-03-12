package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.drive.BulkMecanumDrive;
import org.firstinspires.ftc.teamcode.util.GoalTargetRecognition;

public class AutoAimGoalTask implements RobotControl {
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    GoalTargetRecognition goalRecog;
    BulkMecanumDrive drive;

    public AutoAimGoalTask(RobotHardware robotHardware, RobotProfile robotProfile){
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        this.drive = robotHardware.getMecanumDrive();
        goalRecog = null;
    }

    public String toString() {
        return "AutoAimGoal " + ((goalRecog!=null)?goalRecog:"");
    }

    public boolean isDone() {
        if (goalRecog!=null) {
            return !drive.isBusy();
        }
        else {
            return true;
        }
    }

    public void prepare(){
        RobotVision vision = robotHardware.getRobotVision();
        goalRecog = vision.getGoalTargetRecognition();
        if (goalRecog!=null) {
            double turnAngle = goalRecog.getTargetAngle()-robotProfile.hardwareSpec.shootingAngle;
            int shootVelocity = (int)((goalRecog.getDistanceInch() - robotProfile.hardwareSpec.shootingDistBase) *
                    robotProfile.hardwareSpec.shootingVelocityInch + robotProfile.hardwareSpec.shootVelocity);
            robotHardware.startShootMotor(shootVelocity);
            drive.turnAsync(turnAngle);
        }
    }

    public void execute() {
        if (goalRecog!=null) {
            drive.update();
        }
    }

    public void cleanUp(){
    }
}

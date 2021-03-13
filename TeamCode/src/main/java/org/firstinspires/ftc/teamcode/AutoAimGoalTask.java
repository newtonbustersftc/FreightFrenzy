package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.drive.BulkMecanumDrive;
import org.firstinspires.ftc.teamcode.util.GoalTargetRecognition;

public class AutoAimGoalTask implements RobotControl {
    State state;
    long startTime;
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    GoalTargetRecognition goalRecog;
    BulkMecanumDrive drive;
    boolean done;
    int origShootCount;
    int shootCount;

    enum State{
        TURNING, SHOOTING, WAIT_BETWEEN_SHOOT;
    }

    public AutoAimGoalTask(RobotHardware robotHardware, RobotProfile robotProfile, int shootCount){
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        this.origShootCount = shootCount;
        this.drive = robotHardware.getMecanumDrive();
        goalRecog = null;
    }

    public String toString() {
        return "AutoAimGoal " + ((goalRecog!=null)?goalRecog:"");
    }

    public boolean isDone() {
        if (goalRecog!=null) {
            return done;
        }
        else {
            return true;
        }
    }

    public void prepare(){
        RobotVision vision = robotHardware.getRobotVision();
        goalRecog = vision.getGoalTargetRecognition();
        if (goalRecog!=null) {
            robotHardware.ringHolderUp();
            double turnAngle = goalRecog.getTargetAngle()-robotProfile.hardwareSpec.shootingAngle;
            int shootVelocity = (int)((goalRecog.getDistanceInch() - robotProfile.hardwareSpec.shootingDistBase) *
                    robotProfile.hardwareSpec.shootingVelocityInch + robotProfile.hardwareSpec.shootVelocity);
            robotHardware.startShootMotor(shootVelocity);
            drive.turnAsync(turnAngle);
            state = State.TURNING;
            shootCount = origShootCount;
            done = false;

        } else {
            done = true;
        }
    }

    public void execute() {
        if (goalRecog!=null) {
            if(state == State.TURNING){
                drive.update();
                if(!drive.isBusy()){
                    state = State.SHOOTING;
                    startTime = System.currentTimeMillis();
                    robotHardware.setShooterPosition(false);
                }
            } else if(state == State.SHOOTING){
                if (System.currentTimeMillis() - startTime > robotProfile.hardwareSpec.shootServoDelay) {
                    robotHardware.setShooterPosition(true);
                    shootCount -= 1;
                    if(shootCount > 0){
                        state = State.WAIT_BETWEEN_SHOOT;
                        startTime = System.currentTimeMillis();
                    }
                    else{
                        done = true;
                    }
                }
            }
            else if (state == State.WAIT_BETWEEN_SHOOT){
                if(System.currentTimeMillis() - startTime > robotProfile.hardwareSpec.shootDelay){
                    state = State.SHOOTING;
                    startTime = System.currentTimeMillis();
                    robotHardware.setShooterPosition(false);
                }
            }
        }
    }

    public void cleanUp(){
    }
}

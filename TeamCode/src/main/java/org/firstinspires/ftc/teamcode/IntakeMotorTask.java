package org.firstinspires.ftc.teamcode;

public class IntakeMotorTask implements RobotControl {
    public enum IntakeMode { NORMAL, REVERSE, STOP };
    transient RobotHardware robot;
    transient RobotProfile profile;
    IntakeMode mode;

    public IntakeMotorTask(RobotHardware robot, RobotProfile profile, IntakeMode mode){
        this.robot = robot;
        this.profile = profile;
        this.mode = mode;
    }

    public String toString() {
        return "Set Intake Mode " + mode;
    }

    public void prepare(){
    }

    public void execute() {
        switch (mode) {
            case NORMAL:
                robot.startIntake();
                break;
            case REVERSE:
                robot.reverseIntake();
                break;
            case STOP:
                robot.stopIntake();
        }
   }

    public void cleanUp(){
    }

    public boolean isDone() {
        return true;
    }
}

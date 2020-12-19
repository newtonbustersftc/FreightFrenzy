package org.firstinspires.ftc.teamcode;

public class ShooterMotorTask implements RobotControl {
    transient RobotHardware robot;
    transient RobotProfile profile;
    boolean isOn;

    public ShooterMotorTask(RobotHardware robot, RobotProfile profile, boolean isOn) {
        this.robot = robot;
        this.profile = profile;
        this.isOn = isOn;
    }

    public String toString() {
        return "Set Shooter Motor ON " + isOn;
    }

    public void prepare(){
        if (isOn) {
            robot.startShootMotor();
        }
        else {
            robot.stopShootMotor();
        }
    }

    public void execute() {
        // already did in prepare
    }

    public void cleanUp(){
    }

    public boolean isDone() {
        return true;
    }
}

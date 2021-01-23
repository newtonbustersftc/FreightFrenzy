package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_RPM;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.rpmToVelocity;

public class ShooterMotorTask implements RobotControl {
    transient RobotHardware robot;
    transient RobotProfile profile;
    transient int velocity;
    boolean isOn;

    public ShooterMotorTask(RobotHardware robot, RobotProfile profile, boolean isOn) {
        this.robot = robot;
        this.profile = profile;
        this.isOn = isOn;
        this.velocity = profile.hardwareSpec.shootVelocity;
    }

    public ShooterMotorTask(RobotHardware robot, RobotProfile profile, boolean isOn, int velocity){
        this.robot = robot;
        this.profile = profile;
        this.isOn = isOn;
        this.velocity = velocity;
    }

    public String toString() {
        return "Set Shooter Motor ON " + isOn;
    }

    public void prepare(){
        if (isOn) {
            robot.startShootMotor(velocity);
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

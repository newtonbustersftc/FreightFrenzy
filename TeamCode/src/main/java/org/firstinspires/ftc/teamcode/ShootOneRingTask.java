package org.firstinspires.ftc.teamcode;

public class ShootOneRingTask implements RobotControl {
    transient RobotHardware robot;
    transient RobotProfile profile;
    long startTime;
    boolean done;

    public ShootOneRingTask(RobotHardware robot, RobotProfile profile) {
        this.robot = robot;
        this.profile = profile;
    }

    public String toString() {
        return "Shoot on Ring";
    }

    public void prepare(){
        startTime = System.currentTimeMillis();
        done = false;
        robot.setShooterPosition(true);
    }

    public void execute() {
        if (System.currentTimeMillis() - startTime > profile.hardwareSpec.shootServoDelay) {
            robot.setShooterPosition(false);
            done = true;
        }
    }

    public void cleanUp(){
    }

    public boolean isDone() {
        return done;
    }
}

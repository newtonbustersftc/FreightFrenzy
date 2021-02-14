package org.firstinspires.ftc.teamcode;

public class RingHolderPosTask implements RobotControl {
    public enum RingHolderPosition {UP, DOWN};

    transient RobotHardware robot;
    transient RobotProfile profile;
    long startTime;
    RingHolderPosition pos;
    boolean done;

    public RingHolderPosTask(RobotHardware robot, RobotProfile profile, RingHolderPosition position) {
        this.robot = robot;
        this.profile = profile;
        this.pos = position;
    }

    public String toString() {
        return "RingHolder to " + pos;
    }

    public void prepare(){
        startTime = System.currentTimeMillis();
        done = false;
    }

    public void execute() {
        if (pos==RingHolderPosition.UP) {
            robot.ringHolderUp();
            robot.setShooterPosition(true);
            robot.setRingPusherPosition(RobotHardware.RingPusherPosition.SHOOT);
        }
        else {
            robot.ringHolderDown();
            robot.setShooterPosition(false);
            robot.setRingPusherPosition(RobotHardware.RingPusherPosition.UP);
        }
    }

    public void cleanUp(){
    }

    public boolean isDone() {
        return System.currentTimeMillis() - startTime > profile.hardwareSpec.shootServoDelay;
    }
}

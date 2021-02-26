package org.firstinspires.ftc.teamcode;

public class RingPusherPosTask implements RobotControl {
    transient RobotHardware robot;
    transient RobotProfile profile;
    long startTime;
    RobotHardware.RingPusherPosition pos;
    boolean done;

    public RingPusherPosTask(RobotHardware robot, RobotProfile profile, RobotHardware.RingPusherPosition position) {
        this.robot = robot;
        this.profile = profile;
        this.pos = position;
    }

    public String toString() {
        return "RingPusher to " + pos;
    }

    public void prepare(){
        startTime = System.currentTimeMillis();
        done = false;
    }

    public void execute() {
        robot.setRingPusherPosition(pos);
        done = true;
    }

    public void cleanUp(){
    }

    public boolean isDone() {
        return done;
        //return System.currentTimeMillis() - startTime > profile.hardwareSpec.shootServoDelay;
    }
}

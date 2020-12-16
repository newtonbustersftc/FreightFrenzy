package org.firstinspires.ftc.teamcode;

public class GrabberTask implements RobotControl {
    transient RobotHardware robot;
    transient RobotProfile profile;
    boolean isOpen;
    long timeDuration;
    transient long timeStart;

    public GrabberTask(RobotHardware robot, RobotProfile profile, boolean isOpen, long timeDuration) {
        this.robot = robot;
        this.profile = profile;
        this.isOpen = isOpen;
        this.timeDuration =  timeDuration;
    }

    public String toString() {
        return "Grabber " + (isOpen?"Open":"Close");
    }

    public void prepare(){
        timeStart = System.currentTimeMillis();
    }

    public void execute() {
        robot.setGrabberPosition(isOpen);
    }

    public void cleanUp(){
    }

    public boolean isDone() {
        return (System.currentTimeMillis()-timeStart) > timeDuration;
    }
}

package org.firstinspires.ftc.teamcode;

public class MoveArmTask implements RobotControl {
    transient RobotHardware robot;
    transient RobotProfile profile;
    RobotHardware.ArmPosition armPosition;
    long timeDuration;
    transient long timeStart;

    public MoveArmTask(RobotHardware robot, RobotProfile profile, RobotHardware.ArmPosition armPosition, long timeDuration) {
        this.robot = robot;
        this.profile = profile;
        this.armPosition = armPosition;
        this.timeDuration =  timeDuration;
    }

    public String toString() {
        return "Move Arm to " + armPosition;
    }

    public void prepare(){
        timeStart = System.currentTimeMillis();
    }

    public void execute() {
        robot.setArmMotorPos(armPosition);
    }

    public void cleanUp(){
    }

    public boolean isDone() {
        return (System.currentTimeMillis()-timeStart) > timeDuration;
    }
}

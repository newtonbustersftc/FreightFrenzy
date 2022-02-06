package org.firstinspires.ftc.teamcode;

public class DeliverToHubTask implements RobotControl {
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    long startTime;
    boolean isAutonomous;

    public DeliverToHubTask(RobotHardware robotHardware, RobotProfile profile){
        this.robotHardware = robotHardware;
        this.robotProfile = profile;
        this.isAutonomous = false;
    }

    public DeliverToHubTask(RobotHardware robotHardware, RobotProfile profile, boolean isAutonomous){
        this.robotHardware = robotHardware;
        this.robotProfile = profile;
        this.isAutonomous = true;
    }

    @Override
    public void prepare() {
        robotHardware.openBoxFlap();
        Logger.logFile("box open");
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
    }

    @Override
    public void cleanUp() {
        if (!isAutonomous) {
            robotHardware.closeBoxFlap();
            robotHardware.setLiftPosition(RobotHardware.LiftPosition.ONE);
        }
//        robotHardware.keepLidMid();
    }

    @Override
    public boolean isDone() {
        return System.currentTimeMillis()-startTime>1000;
    }
}

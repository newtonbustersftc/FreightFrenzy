package org.firstinspires.ftc.teamcode;

public class DeliverToHubTask implements RobotControl {
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    long startTime;

    public DeliverToHubTask(RobotHardware robotHardware, RobotProfile profile){
        this.robotHardware = robotHardware;
        this.robotProfile = profile;
    }

    @Override
    public void prepare() {
        robotHardware.openBoxFlap();
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
    }

    @Override
    public void cleanUp() {
        robotHardware.closeBoxFlap();
    }

    @Override
    public boolean isDone() {
        return System.currentTimeMillis()-startTime>robotProfile.hardwareSpec.duckSpinTime;
    }
}
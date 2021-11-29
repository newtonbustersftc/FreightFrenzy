package org.firstinspires.ftc.teamcode;

public class IntakeTask implements RobotControl {
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    long startTime;

    public IntakeTask(RobotHardware robotHardware, RobotProfile profile){
        this.robotHardware = robotHardware;
        this.robotProfile = profile;
    }

    @Override
    public void prepare() {
        robotHardware.startIntake();
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
    }

    @Override
    public void cleanUp() {
        robotHardware.stopIntake();
    }

    @Override
    public boolean isDone() {
        return System.currentTimeMillis()-startTime>robotProfile.hardwareSpec.intakeTime;
    }
}

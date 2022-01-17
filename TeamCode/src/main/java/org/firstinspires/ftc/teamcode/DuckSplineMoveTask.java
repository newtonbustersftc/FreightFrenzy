package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class DuckSplineMoveTask implements RobotControl {
    RobotHardware hardware;
    SampleMecanumDrive drive;
    Trajectory trajectory;
    int currDuckEncoder;

    public DuckSplineMoveTask(SampleMecanumDrive drive, Trajectory trajectory, RobotHardware hardware){
        this.drive = drive;
        this.trajectory = trajectory;
        this.hardware = hardware;
    }
    @Override
    public void prepare() {
        drive.followTrajectoryAsync(trajectory);
        hardware.ogDuckEncoder = hardware.getEncoderCounts(RobotHardware.EncoderType.DUCK);
    }

    @Override
    public void execute() {
        currDuckEncoder = hardware.getEncoderCounts(RobotHardware.EncoderType.DUCK);
        drive.update();
    }

    @Override
    public void cleanUp() {
        hardware.setMotorPower(0, 0, 0, 0);
    }

    @Override
    public boolean isDone() {
        return !drive.isBusy() || hardware.ogDuckEncoder - currDuckEncoder != 0;
    }
}

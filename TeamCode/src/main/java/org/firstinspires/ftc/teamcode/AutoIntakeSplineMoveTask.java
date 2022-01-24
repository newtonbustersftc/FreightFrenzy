package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutoIntakeSplineMoveTask implements RobotControl {
    RobotHardware hardware;
    SampleMecanumDrive drive;
    Trajectory trajectory;
    RobotHardware.Freight freight;

    public AutoIntakeSplineMoveTask(Trajectory trajectory, RobotHardware hardware){
        this.drive = hardware.mecanumDrive;
        this.trajectory = trajectory;
        this.hardware = hardware;
    }
    @Override
    public void prepare() {
        Logger.logFile("in AutoIntakeSplineMoveTask, prepare");
        drive.followTrajectoryAsync(trajectory);
//        freight = RobotHardware.Freight.NONE;
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void cleanUp() {
        Logger.logFile("AutoIntakeSplineMove clean up");
        hardware.setMotorPower(0, 0, 0, 0);
    }

    @Override
    public boolean isDone() {
        return !drive.isBusy();
    }
}

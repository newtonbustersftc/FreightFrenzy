package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.TreeMap;

public class AutoIntakeMoveTask implements RobotControl {
    RobotHardware hardware;
    SampleMecanumDrive drive;
    Trajectory trajectory;
    TrajectorySequence trajectorySequence;

    public AutoIntakeMoveTask(Trajectory trajectory, RobotHardware hardware){
        this.drive = hardware.mecanumDrive;
        this.hardware = hardware;
        this.trajectory = trajectory;
    }

    public AutoIntakeMoveTask(TrajectorySequence trajectorySequence,RobotHardware hardware){
        this.drive = hardware.mecanumDrive;
        this.hardware = hardware;
        this.trajectorySequence = trajectorySequence;
    }
    @Override
    public void prepare() {
        Logger.logFile("in AutoIntakeSplineMoveTask, prepare");
        if(trajectory != null) {
            drive.followTrajectoryAsync(trajectory);
        }else{
           drive.followTrajectorySequenceAsync(trajectorySequence);
        }
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

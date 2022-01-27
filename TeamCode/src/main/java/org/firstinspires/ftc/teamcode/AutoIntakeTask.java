package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutoIntakeTask implements RobotControl{
    enum Mode {
        LIFT, INTAKE, LID, REVERSE, DONE
    }
    Mode mode;
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    long startTime, lidTime, cleanUpTime;
    long timeOut;
    RobotHardware.Freight freight;
    RobotHardware.LiftPosition endLiftPos = RobotHardware.LiftPosition.ONE;
    boolean liftImmediately;

    public AutoIntakeTask(RobotHardware hardware, RobotProfile robotProfile, long timeOut, boolean liftImmediately){
        this.robotHardware = hardware;
        this.robotProfile = robotProfile;
        this.timeOut = timeOut;
        this.liftImmediately = liftImmediately;
        Logger.logFile("in AutoIntakeTask");
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        mode = Mode.LIFT;
        // first move the lift down
        robotHardware.setLiftPosition(RobotHardware.LiftPosition.ZERO);
        Logger.logFile("in AutoIntake prepare, lift position: "+robotHardware.getCurrLiftPos());
    }

    @Override
    public void execute() {
        if (mode==Mode.LIFT) {
            if (robotHardware.getEncoderCounts(RobotHardware.EncoderType.LIFT) < robotProfile.hardwareSpec.liftPositionOne) {
                Logger.logFile("Starting intake.");
                robotHardware.startIntake();
                mode = Mode.INTAKE;
            }
        }
        if (mode==Mode.INTAKE) {
            freight = robotHardware.getFreight();
            if (RobotHardware.Freight.NONE!=freight) {
                robotHardware.closeLid();
                robotHardware.reverseIntake();
                lidTime = System.currentTimeMillis();
                mode = Mode.LID;
                Logger.logFile("AutoIntakeTask: there is fright: " + freight);
            }
        }
        else if (mode==Mode.LID){
            if (System.currentTimeMillis()-lidTime>200) {
                freight = robotHardware.getFreight();
                if (RobotHardware.Freight.NONE!=freight) {
                    mode = Mode.REVERSE;
                    cleanUpTime = System.currentTimeMillis();
                    robotHardware.setLiftPosition(RobotHardware.LiftPosition.BOTTOM);
                    Logger.logFile("AIT: reverse intake");
                }
                else {
                    robotHardware.openLid();
                    Logger.logFile("Intake bounced out");
                    robotHardware.startIntake();
                    mode = Mode.INTAKE;
                }
            }
        }
    }

    @Override
    public void cleanUp() {
        Logger.logFile("autoIntakeTask clean up");
        if(liftImmediately)
            robotHardware.stopIntake(RobotHardware.LiftPosition.TOP);
        else
            robotHardware.stopIntake(RobotHardware.LiftPosition.ONE);
    }

    @Override
    public boolean isDone() {
        if (mode==Mode.REVERSE) {
            return true;
        }
        else {
            if(System.currentTimeMillis()-startTime>timeOut)
                Logger.logFile("time out, exit");
            return System.currentTimeMillis()-startTime>timeOut;
        }
    }
}

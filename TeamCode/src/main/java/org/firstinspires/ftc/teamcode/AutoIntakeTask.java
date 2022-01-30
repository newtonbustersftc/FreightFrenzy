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
        LIFT, INTAKE, LID, REVERSE, JAM, DONE
    }
    Mode mode;
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    long startTime, lidTime, jamTime, cleanUpTime;
    boolean checkIntakeVelo;
    long intakeSlowDetectTime;
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
                checkIntakeVelo = false;
                mode = Mode.INTAKE;
            }
        }
        if (mode==Mode.INTAKE) {
            freight = robotHardware.getFreight();
            if (RobotHardware.Freight.NONE!=freight) {
                robotHardware.keepLidMid(); // to allow 2nd cube to drop if there
                robotHardware.reverseIntake();
                lidTime = System.currentTimeMillis();
                mode = Mode.LID;
                Logger.logFile("AutoIntakeTask: there is freight: " + freight);
            }
            else {
                int intakeVelo = robotHardware.getEncoderVelocity(RobotHardware.EncoderType.INTAKE);
                if (checkIntakeVelo==false) {
                    if (intakeVelo > robotProfile.hardwareSpec.intakeVelocity) {
                        // once pass the threshold, then we start looking for slow down
                        checkIntakeVelo = true;
                        intakeSlowDetectTime = -1;
                    }
                }
                else {
                    if (intakeVelo < robotProfile.hardwareSpec.intakeVelocity) {
                        if (intakeSlowDetectTime==-1) {
                            // initial detection
                            intakeSlowDetectTime = System.currentTimeMillis();
                        }
                        else {
                            jamTime = System.currentTimeMillis();
                            if (jamTime-intakeSlowDetectTime>1500) {
                                // jam for 1 second
                                robotHardware.reverseIntake();
                                mode = Mode.JAM;
                                checkIntakeVelo = false;
                                Logger.logFile("AutoIntake jam detected");
                            }
                        }
                    }
                    else {
                        intakeSlowDetectTime = -1;  // reset the jam clock
                    }
                }
            }
        }
        else if (mode==Mode.LID){
            if (System.currentTimeMillis()-lidTime>200) {
                freight = robotHardware.getFreight();
                if (RobotHardware.Freight.NONE!=freight) {
                    mode = Mode.REVERSE;
                    cleanUpTime = System.currentTimeMillis();
                    robotHardware.closeLid();
                    robotHardware.setLiftPosition(RobotHardware.LiftPosition.BOTTOM);
                    Logger.logFile("AIT: reverse intake");
                }
                else {
                    robotHardware.openLid();
                    Logger.logFile("Intake bounced out");
                    robotHardware.startIntake();
                    checkIntakeVelo = false;
                    mode = Mode.INTAKE;
                }
            }
        }
        else if (mode==Mode.JAM) {
            if (System.currentTimeMillis()-jamTime>500) {
                mode = Mode.INTAKE;
                robotHardware.startIntake();
                checkIntakeVelo = false;
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

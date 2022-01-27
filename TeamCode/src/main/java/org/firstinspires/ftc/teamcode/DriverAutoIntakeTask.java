package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class DriverAutoIntakeTask implements RobotControl{
    enum Mode {
        LIFT, INTAKE, LID, CHECK, REVERSE, DONE
    }
    Mode mode;
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    long startTime, cleanUpTime, lidTime;
    long timeOut;
    Gamepad gamepad;
    RobotHardware.Freight freight;
    RobotHardware.LiftPosition endLiftPos = RobotHardware.LiftPosition.ONE;
    SampleMecanumDrive drive;
    Pose2d nextPos;

    public DriverAutoIntakeTask(RobotHardware hardware, RobotProfile robotProfile, long timeOut, SampleMecanumDrive drive, Pose2d nextPos){
        this.robotHardware = hardware;
        this.robotProfile = robotProfile;
        this.timeOut = timeOut;
        this.drive = drive;
        this.nextPos = nextPos;
        Logger.logFile("in AutoIntakeTask");
    }

    public DriverAutoIntakeTask(RobotHardware hardware, RobotProfile robotProfile, Gamepad gamepad) {
        this.robotHardware = hardware;
        this.robotProfile = hardware.getRobotProfile();
        this.timeOut = 0;
        this.gamepad = gamepad;
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        mode = Mode.LIFT;
        // first move the lift down
        robotHardware.setLiftPosition(RobotHardware.LiftPosition.ZERO);
        Logger.logFile("in AUtoIntake prepare, lift position: "+robotHardware.getCurrLiftPos());
    }

    @Override
    public void execute() {
        if (mode==Mode.LIFT) {
            if (robotHardware.getEncoderCounts(RobotHardware.EncoderType.LIFT) < robotProfile.hardwareSpec.liftPositionOne) {
                Logger.logFile("Starting intake.");
                robotHardware.startIntake();
                mode = Mode.INTAKE;
            } else if (gamepad.right_trigger == 0){
                cleanUpTime = System.currentTimeMillis();
                mode = Mode.REVERSE;
                setEndLiftPos();
            }
        } else if (mode == Mode.INTAKE){
            freight = robotHardware.getFreight();
            if (RobotHardware.Freight.NONE!=freight){
                robotHardware.keepLidMid(); // not fully close, so 2nd brick can fall out
                robotHardware.reverseIntake();
                mode = Mode.LID;
                lidTime = System.currentTimeMillis();
            } else if (gamepad.right_trigger == 0){
                cleanUpTime = System.currentTimeMillis();
                mode = Mode.REVERSE;
                setEndLiftPos();
            }
        } else if (mode == Mode.LID){
            if(System.currentTimeMillis() - lidTime > 200){
                freight = robotHardware.getFreight();
                if (RobotHardware.Freight.NONE != freight){
                    mode = Mode.REVERSE;
                    robotHardware.closeLid();
                    robotHardware.reverseIntake(endLiftPos);
                    setEndLiftPos();
                    cleanUpTime = System.currentTimeMillis();
                } else {
                    mode = Mode.INTAKE;
                    robotHardware.openLid();
                    Logger.logFile("Intake bounced out");
                    robotHardware.startIntake();
                }
            }
        }
    }

    void setEndLiftPos() {
        if (freight==RobotHardware.Freight.NONE) {
            endLiftPos = RobotHardware.LiftPosition.ONE;
        }
        else {
            endLiftPos = RobotHardware.LiftPosition.BOTTOM;
        }
    }

    @Override
    public void cleanUp() {
        robotHardware.stopIntake(endLiftPos);
        if (endLiftPos == RobotHardware.LiftPosition.ONE) {
            robotHardware.keepLidMid();
        }
    }

    @Override
    public boolean isDone() {
        if (mode==Mode.REVERSE) {
            return System.currentTimeMillis() - cleanUpTime > 200;
        }
        else {
            return false;
        }
    }
}

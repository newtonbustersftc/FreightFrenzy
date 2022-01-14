package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robot.Robot;

public class AutoIntakeTask implements RobotControl{
    enum Mode {
        INTAKE, REVERSE
    }
    Mode mode;
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    long startTime, cleanUpTime;
    long timeOut;
    Gamepad gamepad;
    RobotHardware.Freight freight;
    RobotHardware.LiftPosition endLiftPos = RobotHardware.LiftPosition.ONE;


    public AutoIntakeTask(RobotHardware hardware, RobotProfile robotProfile, long timeOut){
        this.robotHardware = hardware;
        this.robotProfile = robotProfile;
        this.timeOut = timeOut;
        Logger.logFile("in AutoIntakeTask");
    }

    public AutoIntakeTask(RobotHardware hardware, RobotProfile robotProfile, Gamepad gamepad) {
        this.robotHardware = hardware;
        this.robotProfile = hardware.getRobotProfile();
        this.timeOut = 0;
        this.gamepad = gamepad;
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        mode = Mode.INTAKE;
        // first move the lift down
        robotHardware.setLiftPosition(RobotHardware.LiftPosition.ZERO);
        Logger.logFile("in AUtoIntake prepare, lift position: "+robotHardware.getCurrLiftPos());
    }

    @Override
    public void execute() {
        if (mode==Mode.INTAKE) {
            if (robotHardware.getEncoderCounts(RobotHardware.EncoderType.LIFT) < robotProfile.hardwareSpec.liftPositionOne) {
                Logger.logFile("Starting intake.");
                robotHardware.startIntake();
            }
            freight = robotHardware.gotFreight();
            if ((RobotHardware.Freight.NONE!=freight) ||
                    (timeOut > 0 && System.currentTimeMillis() - startTime > timeOut) ||
                    (timeOut == 0 && gamepad.right_trigger == 0)) {
                Logger.logFile("Reverse intake - freigt: " + freight);
                setEndLiftPos();
                mode = Mode.REVERSE;
                robotHardware.reverseIntake(endLiftPos);
                cleanUpTime = System.currentTimeMillis();
            }
        }
        else {
            RobotHardware.Freight f = robotHardware.gotFreight();
            if (f!=freight) {
                freight = f;
                setEndLiftPos();
                robotHardware.setLiftPosition(endLiftPos);
            }
        }
    }

    void setEndLiftPos() {
        if (freight==RobotHardware.Freight.BALL) {
            endLiftPos = RobotHardware.LiftPosition.TOP;
        }
        else if (freight==RobotHardware.Freight.CUBE) {
            endLiftPos = RobotHardware.LiftPosition.BOTTOM;
        }
        else {
            endLiftPos = RobotHardware.LiftPosition.ONE;
        }
    }

    @Override
    public void cleanUp() {
        robotHardware.stopIntake(endLiftPos);
    }

    @Override
    public boolean isDone() {
        if (mode==Mode.REVERSE) {
            return System.currentTimeMillis() - cleanUpTime > 500;
        }
        else {
            return false;
        }
    }
}

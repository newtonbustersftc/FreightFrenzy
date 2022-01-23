package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutoIntakeTask implements RobotControl{
    enum Mode {
        INTAKE, REVERSE, DONE
    }
    Mode mode;
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    long startTime, cleanUpTime;
    long timeOut;
    Gamepad gamepad;
    RobotHardware.Freight freight;
    RobotHardware.LiftPosition endLiftPos = RobotHardware.LiftPosition.ONE;
    SampleMecanumDrive drive;
    Pose2d nextPos;

    public AutoIntakeTask(RobotHardware hardware, RobotProfile robotProfile, long timeOut, Pose2d nextPos){
        this.robotHardware = hardware;
        this.robotProfile = robotProfile;
        this.timeOut = timeOut;
        this.drive = hardware.mecanumDrive;
        this.nextPos = nextPos;
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
            freight = robotHardware.getFreight();
            if (RobotHardware.Freight.NONE!=freight){
                if(timeOut == 0 && gamepad.right_trigger == 0) {
                    Logger.logFile("Reverse intake - freigt: " + freight);
                    setEndLiftPos();
                }else if(timeOut > 0 ){  //this is autonomous
                    Logger.logFile("autonomous, autointake get - freigt: " + freight);
                    mode = Mode.REVERSE;
                    robotHardware.reverseIntake(endLiftPos);
                    cleanUpTime = System.currentTimeMillis();
                }
            }
        }
        else {
            RobotHardware.Freight f = robotHardware.getFreight();
            if (f!=freight) {
                freight = f;
                setEndLiftPos();
                robotHardware.setLiftPosition(endLiftPos);
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
        if(timeOut == 0){//driver opmode only
            robotHardware.stopIntake(endLiftPos);
            if (endLiftPos == RobotHardware.LiftPosition.ONE) {
                robotHardware.keepLidMid();
            }
        }else{ //autonomous
            TrajectoryAccelerationConstraint slowAccConstraint = SampleMecanumDrive.getAccelerationConstraint((25));
            TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(10, 10, 10.25);

            Logger.logFile("autoIntakeTask clean up");
            Pose2d curPos = robotHardware.getLocalizer().getPoseEstimate();
            Logger.logFile("pick up pose in AutoIntakeTask: " + curPos);

            robotHardware.stopIntake(RobotHardware.LiftPosition.TOP);
            if(drive !=null && nextPos!=null) {   //this is only for ParallelComboIntakeMovePriority
                Trajectory traj = drive.trajectoryBuilder(curPos, true)
                        .splineTo(nextPos.vec(), nextPos.getHeading() + Math.PI, slowVelConstraints, slowAccConstraint)
                        .build();
                drive.followTrajectoryAsync(traj);
            }
            robotHardware.stopIntake(RobotHardware.LiftPosition.TOP);
        }
    }

    @Override
    public boolean isDone() {
        if (mode==Mode.REVERSE) {
//            return System.currentTimeMillis() - cleanUpTime > 500;
            return true;
        }
        else {
            return false;
        }
    }
}

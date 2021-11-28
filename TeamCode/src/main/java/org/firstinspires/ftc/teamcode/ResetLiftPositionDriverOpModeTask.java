package org.firstinspires.ftc.teamcode;

public class ResetLiftPositionDriverOpModeTask implements RobotControl{
    RobotHardware robotHardware;

    public ResetLiftPositionDriverOpModeTask(RobotHardware robotHardware){
        this.robotHardware = robotHardware;
    }

    @Override
    public void prepare() {
        robotHardware.setLiftMotorPosition(-3000);
    }

    @Override
    public void execute() {
    }

    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        if(robotHardware.liftBottomTouched()){
            robotHardware.resetLiftEncoderCount();
            robotHardware.setLiftPosition(RobotHardware.LiftPosition.ZERO);
            return true;
        }
        return false;
    }
}

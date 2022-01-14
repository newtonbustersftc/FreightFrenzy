package org.firstinspires.ftc.teamcode;

public class ResetLiftPositionDriverOpModeTask implements RobotControl{
    RobotHardware robotHardware;
    boolean setPos;

    public ResetLiftPositionDriverOpModeTask(RobotHardware robotHardware){
        this.robotHardware = robotHardware;
    }

    @Override
    public void prepare() {
        robotHardware.closeLid();
        robotHardware.closeBoxFlap();
        setPos = false;
    }

    @Override
    public void execute() {
        if (!setPos) {
            robotHardware.setLiftMotorPosition(-3000, 0.1);
            setPos = true;
        }
    }

    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        if(robotHardware.liftBottomTouched()){
            robotHardware.resetLiftEncoderCount();
            robotHardware.setLiftPosition(RobotHardware.LiftPosition.ONE);
            return true;
        }
        return false;
    }
}

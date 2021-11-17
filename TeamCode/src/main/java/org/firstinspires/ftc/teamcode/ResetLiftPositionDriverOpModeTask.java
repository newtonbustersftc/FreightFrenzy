package org.firstinspires.ftc.teamcode;

public class ResetLiftPositionDriverOpModeTask implements RobotControl{
    RobotHardware robotHardware;

    public ResetLiftPositionDriverOpModeTask(RobotHardware robotHardware){
        this.robotHardware = robotHardware;
    }

    @Override
    public void prepare() {

    }

    @Override
    public void execute() {
        if(robotHardware.getCurrLiftPos() == RobotHardware.LiftPosition.NOT_INIT &&
                !robotHardware.liftBottomTouched()){
            int currLiftPos = robotHardware.getEncoderCounts(RobotHardware.EncoderType.LIFT);
            Logger.logFile("Lift current Position " + currLiftPos);
            if(Math.abs(currLiftPos)>400)
                robotHardware.setLiftMotorPosition(currLiftPos -300);
            else
                robotHardware.setLiftMotorPosition(currLiftPos - 30);
        }

    }

    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        if(robotHardware.getCurrLiftPos() == RobotHardware.LiftPosition.NOT_INIT &&
                robotHardware.liftBottomTouched()){
            //rise up a bit to release the tension between lift motor and touch sensor
            //after testing, the average range of encoder counts when the touch sensor is
            //touched at bottom and touched at top is about 10 counts
            Logger.logFile("just want to prove we are here, cnt=" + robotHardware.liftMotor.getCurrentPosition());
           // robotHardware.setLiftMotorPosition(robotHardware.getEncoderCounts(RobotHardware.EncoderType.LIFT) + 20);
            try{
                Thread.sleep(100);
            }catch (Exception e){
                System.out.println(e.getStackTrace());
            }
            Logger.logFile(" now cnt = " + robotHardware.liftMotor.getCurrentPosition());
            robotHardware.resetLiftEncoderCount();
            robotHardware.setLiftPosition(RobotHardware.LiftPosition.ZERO);
            return true;
        }
        return false;
    }
}

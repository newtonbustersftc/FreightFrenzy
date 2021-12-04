package org.firstinspires.ftc.teamcode;

public class AutoIntakeTask implements RobotControl{

    RobotHardware robotHardware;
    long begin;
    long preStop;
    enum IntakeMode { WAIT_FOR_BLOCK, WAIT_FOR_SPEED, STOP}
    AutoIntakeTask.IntakeMode currIntakeMode;

    public AutoIntakeTask(RobotHardware hardware){
        this.robotHardware = hardware;
    }

    @Override
    public void prepare() {
        begin = System.currentTimeMillis();
        robotHardware.startIntake();
        currIntakeMode = IntakeMode.WAIT_FOR_BLOCK;
    }
    //.5 mils after start if speed < 800 then block has been intaked
    //after speed go back to 2000 then stop intake

    @Override
    public void execute() {
        if(currIntakeMode == IntakeMode.WAIT_FOR_BLOCK){
            if((System.currentTimeMillis() - begin) > 500){
                if(robotHardware.getEncoderVelocity(RobotHardware.EncoderType.INTAKE) < 1200){
                    currIntakeMode = IntakeMode.WAIT_FOR_SPEED;
                }
            }
        } else if (currIntakeMode == IntakeMode.WAIT_FOR_SPEED){
            if(robotHardware.getEncoderVelocity(RobotHardware.EncoderType.INTAKE) > 2000){
                currIntakeMode = IntakeMode.STOP;
                preStop = System.currentTimeMillis();
            }
        }
        //Logger.logFile("intakeEncoder: " + robotHardware.getEncoderVelocity(RobotHardware.EncoderType.INTAKE));
    }

    @Override
    public void cleanUp() {
        robotHardware.stopIntake();
    }

    @Override
    public boolean isDone() {
        return (currIntakeMode == IntakeMode.STOP && (System.currentTimeMillis()-preStop>10));
    }
}

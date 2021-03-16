package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class ShootRingTask implements RobotControl {
    enum State { SHOOT, WAIT };
    transient RobotHardware robot;
    transient RobotProfile profile;
    transient OpMode opMode;
    transient State state;
    long startTime;

    public ShootRingTask(RobotHardware robot, RobotProfile profile, OpMode opMode) {
        this.robot = robot;
        this.profile = profile;
        this.opMode = opMode;
        this.startTime = 0;
    }

    public String toString() {
        return "Shoot Rings";
    }

    public void prepare(){
        long nowTime = System.currentTimeMillis();
        if (nowTime-startTime<profile.hardwareSpec.shootDelay) {
            state = State.WAIT;
        }
        else {
            startTime = nowTime;
            state = State.SHOOT;
            Logger.logFile("Shoot with velocity:" + robot.getEncoderVelocity(RobotHardware.EncoderType.SHOOTER));
            robot.setShooterPosition(false);
        }
    }

    public void execute() {
        if (state==State.SHOOT) {
            if (System.currentTimeMillis() - startTime > profile.hardwareSpec.shootServoDelay) {
                state = State.WAIT;
                robot.setShooterPosition(true);
                startTime = System.currentTimeMillis();
            }
        }
        else if (System.currentTimeMillis() - startTime > profile.hardwareSpec.shootDelay) {
            state = State.SHOOT;
            Logger.logFile("Shoot with velocity:" + robot.getEncoderVelocity(RobotHardware.EncoderType.SHOOTER));
            robot.setShooterPosition(false);
            startTime = System.currentTimeMillis();
        }
    }

    public void cleanUp(){
    }

    public boolean isDone() {
        return state==State.WAIT && !opMode.gamepad1.x;
    }
}

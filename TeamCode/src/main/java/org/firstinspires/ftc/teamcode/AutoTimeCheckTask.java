package org.firstinspires.ftc.teamcode;

public class AutoTimeCheckTask implements RobotControl{

    long timeRemaining, startTime, currTime;
    RobotHardware robotHardware;
    RobotControl taskA, taskB;
    boolean aTrue;

    public AutoTimeCheckTask(long startTime, long timeRemaining, RobotHardware robotHardware, RobotControl taskA, RobotControl taskB){
        this.startTime = startTime;
        this.timeRemaining = timeRemaining;
        this.robotHardware = robotHardware;
        this.taskA = taskA;
        this.taskB = taskB;
    }

    @Override
    public void prepare() {
        currTime = System.currentTimeMillis();

        if(Math.abs(currTime - startTime) < timeRemaining){
            aTrue = true;
            taskA.prepare();
        } else {
            aTrue = false;
            taskB.prepare();
        }
    }

    @Override
    public void execute() {
        if(aTrue){
            taskA.execute();
        } else {
            taskB.execute();
        }
    }

    @Override
    public void cleanUp() {
        if (aTrue) {
            taskA.cleanUp();
        } else {
            taskB.cleanUp();
        }
    }

    @Override
    public boolean isDone() {
        return aTrue ? taskA.isDone() : taskB.isDone();
    }
}
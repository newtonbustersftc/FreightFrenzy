package org.firstinspires.ftc.teamcode;

import com.spartronics4915.lib.T265Camera;

public class T265IgnoreConfidenceLevelTask implements RobotControl{
    boolean isIgnoreConfidenceLevel;
    RobotHardware hardware;

    public T265IgnoreConfidenceLevelTask(RobotHardware hardware, boolean isIgnoreConfidenceLevel) {
        this.hardware = hardware;
        this.isIgnoreConfidenceLevel = isIgnoreConfidenceLevel;
    }

    @Override
    public void prepare() {
        Logger.logFile("start ignore T265 confidence level");
    }

    @Override
    public void execute() {
        hardware.setIgnoreT265Confidence(isIgnoreConfidenceLevel);
    }


    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return true;
    }

    public String toString() {
        return "IgnoreT265ConfidenceTask: " + isIgnoreConfidenceLevel;
    }
}

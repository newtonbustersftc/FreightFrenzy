package org.firstinspires.ftc.teamcode;

public class AutoTimeManager {
    static long startTime;

    AutoTimeManager(){
        startTime = System.currentTimeMillis();
    }

    public static long getStartTime(){
        return startTime;
    }
}

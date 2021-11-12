package org.firstinspires.ftc.teamcode;

public class DuckCarouselSpinTask implements RobotControl {
    int alliance;
    RobotHardware robotHardware;
    boolean isBeginning;
    long begin;

    public DuckCarouselSpinTask(RobotHardware hardware, int teamAlliance){  //1 is RED, -1 is BLUE
        this.robotHardware = hardware;
        this.alliance = teamAlliance;
    }

    @Override
    public void prepare() {
        begin = System.currentTimeMillis();
        robotHardware.startDuck(alliance);
    }

    @Override
    public void execute() {
    }

    @Override
    public void cleanUp() {
        robotHardware.stopDuck();
    }

    @Override
    public boolean isDone() {
        if((System.currentTimeMillis() - begin) > 5000){
            return true;
        }
        return false;
    }
}
